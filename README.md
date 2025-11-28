# Mcvilloid / mcvilloid – Controlador de marcha y balance

Este proyecto implementa un **controlador completo de marcha y balance** para el robot humanoide a futuro Mcvilloid en Webots, con una arquitectura modular pensada para:

- Separar **generación de marcha** de **control de balance**.
- Tener una **FSM global de estados** (STAND, PRELEAN, WALK, RECOVER, POSE).
- Incorporar **mecanismos de seguridad** (clamps, detección de pasos malos, recuperación).
- Exponer controles simples por teclado para probar el robot en simulación.

---

## 1. Estructura general

Las piezas principales del controlador son:

- `mcvilloid_controller.py`  
  Bucle principal:
  - Lee sensores (IMU, gyro, GPS).
  - Mantiene la FSM global de estados del robot.
  - Llama al controlador de balance y al generador de marcha.
  - Aplica límites articulares y lógica de seguridad.
  - Envía las referencias finales a los motores.

- **Balance**  
  - `balance/balance.py` → `BalanceController` (control PD(+I) en pitch/roll).
  - `balance/filters.py` → filtros pasa-bajas 1er orden, deadband y soft-start.
  - `balance_shaping.py` → recortes y límites finos en tobillos.

- **Generación de marcha (Walker)**  
  - `movement/walking.py` → clase `Walker` (FSM: STANCE_L → SWING_R → STANCE_R → SWING_L).
  - `gait_shaping.py` → escalado de offsets, matching de rodillas, stride_gain y toques de COM.

- **IMU y estabilidad**  
  - `imu_stability.py` → filtrado de IMU y acumulador de “tiempo estable”.

- **FSMs auxiliares**  
  - `step_fsm.py` → calidad de pasos; dispara RECOVER si hay muchos pasos malos seguidos.
  - `pose_fsm.py` → FSM para la pose estática de levantar pierna (F/G/H).
  - `recover_step` (función suelta) → ley de control en modo RECOVER.

- **Seguridad y límites**  
  - `safety_limits.py` → clamps con contexto, detección de “clamp sostenido” en tobillos.
  - `joint_limits.py` → límites por tipo de junta + posible transición a RECOVER.

- **Telemetría y logging**  
  - `rate_logger.py` → logger con rate limit y canales.
  - `telemetry.py` → log periódico de estado, IMU, juntas y GPS.

- **Teclado / control manual**  
  - `controls.py` → mapeo de teclas a:
    - ON/OFF de marcha.
    - Dirección (adelante / atrás).
    - Pose de levantar pierna.
    - Ajuste de zancada base.

- **Launch de Webots desde ROS2**  
  - `launch/mi_robot_launch.py` → arranca Webots con el mundo correspondiente
    usando `WEBOTS_HOME` y conversión de ruta si se ejecuta en WSL + webots.exe.

---

## 2. Flujo de ejecución (alto nivel)

En cada ciclo del `mcvilloid_controller` ocurre algo parecido a esto:

1. **Lectura de sensores**
   - IMU → roll, pitch, yaw.
   - Gyro → velocidades angulares (sobre todo `gy` para pitch).
   - GPS → posición (x, z) para medir avance.

2. **Filtrado de IMU**
   - `imu_update(...)` produce versiones filtradas `pitch_f`, `roll_f` para decisiones de estabilidad.
   - `update_stable_t(...)` integra un tiempo “estable” basado en |pitch_f| y |roll_f|.

3. **FSM global de estados**
   - `state` puede ser, por ejemplo:
     - `STAND`   → quieto, sólo balance.
     - `PRELEAN` → pequeñas inclinaciones previas a caminar.
     - `WALK`    → marcha activa (Walker ON).
     - `RECOVER` → recuperación ante caída / inestabilidad fuerte.
     - `POSE`    → levantar pierna (controlado por `pose_fsm`).

4. **Balance**
   - `BalanceController.step(dt, pitch, roll)` calcula offsets pequeños para tobillos y caderas en pitch/roll.
   - `compute_balance_offsets(...)` recorta estos offsets, sobre todo en tobillos, según el signo de `pitch_f`.

5. **Generación de marcha**
   - `walker.step(dt, imu, sensors)` devuelve offsets periódicos en cadera/rodilla/tobillo para cada pierna.
   - `apply_gait_offsets(...)`:
     - Escala estos offsets según tipo de joint y `stride_gain`.
     - Hace matching de rodillas izquierda/derecha para evitar asimetrías.

6. **Shaping de marcha**
   - `update_walk_outputs(...)`:
     - Soft-start de la marcha (la ganancia del gait sube poco a poco).
     - Calcula el número de pasos (`steps_taken`).
     - Llama a `step_fsm_update` para clasificar cada paso como bueno/malo.
     - Ajusta:
       - `STRIDE_BASE` (zancada base).
       - `stride_gain` (multiplicador de zancada).
       - `gait_gain` (ganancia global del Walker).
     - Añade toques a caderas y tobillos (toe-off, heel-strike, COM capture, etc.).

7. **Pose estática (si aplica)**
   - `update_pose(dt, pose, target, base_pose, LOG)` aplica la FSM de levantamiento
     de pierna cuando se ha activado con F / G.

8. **Límites y seguridad**
   - `apply_joint_limits_and_clamps(...)`:
     - Limita rango de tobillos, rodillas y caderas.
     - Usa un contexto para detectar “clamp sostenido” en tobillos y disparar RECOVER.
   - `step_fsm_update(...)` puede cambiar `state` a RECOVER si hay demasiados pasos malos.
   - `recover_step(...)` aplica una ley de control específica para “levantar” el robot o devolverlo a una zona segura de pitch.

9. **Telemetría**
   - `periodic_log(...)` imprime cada cierto tiempo:
     - Estado (`state`), ganancia del Walker.
     - IMU (pitch, roll).
     - Valores de algunas joints clave.
     - Posición por GPS.

10. **Salida a motores**
    - Se suman:
      - `base_pose` + offsets de balance + offsets de marcha + correcciones extra.
    - Se escribe el resultado en los motores con `setPosition(...)`.

---

## 3. Controles por teclado (Webots)

Desde `controls.py`:

- **Marcha**
  - `W` → habilitar marcha (`gait_enable = True`).
  - `S` → deshabilitar marcha (`gait_enable = False`).

- **Dirección**
  - `A` → dirección hacia atrás (`walker.dir = -1.0`).
  - `D` → dirección hacia adelante (`walker.dir = +1.0`).

- **Pose estática (levantar pierna)**
  - `F` → levantar pierna izquierda (activa FSM de pose en modo “L”).
  - `G` → levantar pierna derecha.
  - `H` → cancelar pose (vuelve a IDLE).

- **Zancada base**
  - `P` → aumenta `STRIDE_BASE` (hasta `STRIDE_MAX`).
  - `M` → reduce `STRIDE_BASE` (hasta `STRIDE_MIN`).

---

## 4. Integración con ROS2 y Webots

El archivo:

```text
~/ros2_ws/src/mcvilloid/launch/mi_robot_launch.py
