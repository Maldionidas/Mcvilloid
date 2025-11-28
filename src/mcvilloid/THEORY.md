
---

## THEORY.md

```markdown
# Mcvilloid / Mcvilloid – Teoría y fundamentos

Este documento explica la **teoría de control**, las **fórmulas** y las **ideas matemáticas**
que se utilizaron para diseñar el controlador de marcha y balance del robot Mcvilloid.

---

## 1. Modelado básico y notación

- Un robot humanoide simplificado se puede ver como un conjunto de eslabones
  (pies, piernas, torso) con **articulaciones** en caderas, rodillas y tobillos.
- Usamos:
  - radianes para las articulaciones (cadera, rodilla, tobillo).
  - radianes para ángulos de IMU (`pitch`, `roll`, `yaw`).
- El torso se describe principalmente con:
  - `pitch` (inclinación hacia delante/atrás).
  - `roll` (inclinación lateral).

En este robot, por convención:

- `hip_pitch NEGATIVO = torso hacia adelante`.
- `ankle_pitch` se usa como toe-up / toe-down según la geometría.

La **marcha** se representa como un ciclo periódico con una **fase** \(\phi \in [0,1]\)
y una FSM de cuatro fases: STANCE_L → SWING_R → STANCE_R → SWING_L.

---

## 2. Control de balance

### 2.1. Filtro pasa-bajas de primer orden

En `Lowpass` (balance/filters.py) se usa un filtro de 1er orden:

1. Se calcula la constante de tiempo a partir de la frecuencia de corte:

\[
\text{rc} = \frac{1}{2\pi f_c}
\]

2. Se define el factor \(\alpha\):

\[
\alpha = \frac{dt}{\text{rc} + dt}
\]

3. La ecuación en tiempo discreto es:

\[
x[n] = x[n-1] + \alpha \big(u[n] - x[n-1]\big)
\]

Se utiliza para:

- Filtrar la derivada del error (evitar amplificación de ruido).
- Filtrar el comando final (evitar “golpes” bruscos en las articulaciones).

### 2.2. Deadband en grados

La función `deadband_deg(x, band)` aplica un **umbral muerto**:

\[
e_\text{db} =
\begin{cases}
0 & |x| \le \text{band}\\
\operatorname{sign}(x)\,(|x| - \text{band}) & \text{si } |x| > \text{band}
\end{cases}
\]

Esto elimina errores muy pequeños para que el controlador no “tiemble” alrededor del equilibrio.

### 2.3. “Quiet zone”

Además del deadband, la **quiet zone** amplía un poco la zona donde:

- Se pone \(K_d = 0\) (no se calcula derivada).
- Se reduce \(K_p\) (p. ej. multiplicando por 0.65).

Esto suaviza la respuesta cerca del equilibrio, evitando micro-oscillaciones (baile del robot).

### 2.4. Soft-start de ganancia

`SoftStart` implementa una rampa 0..1 en un tiempo \(T\) con un perfil tipo *ease-in-out*:

\[
g = \frac{t}{T}, \quad
g_\text{soft} = g^2(3 - 2g)
\]

Propiedades:

- \(g_\text{soft}(0) = 0\), \(g_\text{soft}(1) = 1\).
- Derivada nula en los extremos (arranque y final suaves).

Se usa para multiplicar el comando de balance al arrancar o reactivar el controlador.

### 2.5. PD(+I) en pitch y roll

Se trabaja con errores en grados:

- \(e_p\) = error en pitch (deg).
- \(e_r\) = error en roll (deg).

Tras aplicar deadband:

- \(e_{p,\text{db}}\),
- \(e_{r,\text{db}}\),

y calculando derivadas discretas:

\[
\dot e_p \approx \frac{e_{p,\text{db}}[n] - e_{p,\text{db}}[n-1]}{dt}
\]

La ley PD(+I) usada es:

\[
u_p^\circ = K_{p,p} e_{p,\text{db}} + K_{d,p} \dot e_p + K_{i,p} I_p
\]
\[
u_r^\circ = K_{p,r} e_{r,\text{db}} + K_{d,r} \dot e_r + K_{i,r} I_r
\]

donde \(I_p\) e \(I_r\) son integrales discretas (acumulación de error) limitadas para evitar wind-up.

Posteriormente se convierten a radianes y se filtran:

\[
u_p = \text{LPF}\big(\text{deg2rad}(u_p^\circ)\big)\,g_\text{soft}
\]
\[
u_r = \text{LPF}\big(\text{deg2rad}(u_r^\circ)\big)\,g_\text{soft}
\]

Este es un PID clásico pero con:

- Filtro derivativo,
- Integrador opcional,
- Soft-start y deadband.

### 2.6. Mezcla ankle/hip

La señal de pitch se reparte entre tobillos y caderas:

\[
u_{ap} = u_p \cdot m_{pa}, \quad u_{hp} = u_p (1 - m_{pa})
\]

La de roll se reparte entre tobillos roll y caderas roll:

\[
u_{ar} = u_r \cdot m_{ra}, \quad u_{hr} = u_r (1 - m_{ra})
\]

Esta mezcla representa una combinación de:

- **Ankle strategy** (ajustar solo tobillos).
- **Hip strategy** (usar caderas para equilibrar el torso).

En humanos se sabe que para perturbaciones grandes entra más la **hip strategy**, por eso aquí se da un rol no despreciable a las caderas.

---

## 3. Generador de marcha (Walker)

### 3.1. FSM de fases y fase local

El Walker usa:

- Un estado discreto: `Phase.STANCE_L`, `SWING_R`, `STANCE_R`, `SWING_L`.
- Una variable de tiempo en cada fase: `_phase_time`.
- Duración de fase: `self._phase_dur = swing_time_s` o `stance_time_s`.

La **fase local** se calcula como:

\[
\phi = \frac{\_phase\_time}{\_phase\_dur}, \quad \phi \in [0,1]
\]

Esa fase se usa para generar trayectorias suaves en las articulaciones.

### 3.2. Trayectorias suaves (senos y parabólicas)

En swing:

- Se usa `s = sin(π φ)` que va 0 → 1 → 0 en el intervalo [0,1]:

\[
s(\phi) = \max\big(0,\sin(\pi \phi)\big)
\]

- La rodilla llega a un máximo en torno a φ ≈ 0.5:

\[
\theta_\text{knee}(\phi) \approx \text{amp} \cdot \theta_\text{knee,max} \cdot s(\phi)
\]

- Para el pie, la función `foot_height`:

\[
z(\phi) = 4 h_\text{max} \phi (1 - \phi)
\]

describe una parábola con máximo en φ=0.5 (altura máxima en el medio del swing).

### 3.3. Dorsiflexión con ventana tipo “campana”

Se usa `bell_window(φ, a, b)`:

\[
w(\phi) =
\begin{cases}
0 & \phi < a \text{ o } \phi > b\\
4 t (1-t) & t = \frac{\phi-a}{b-a}
\end{cases}
\]

Esto crea una “campana” con pico en el centro del intervalo [a,b]. Se aplica a la dorsiflexión:

\[
\theta_\text{ankle}(\phi) \approx \text{amp} \cdot \left( \theta_\text{base} + \theta_\text{dorsi,mid} \cdot w(\phi) \right)
\]

para que el pie levante la punta sólo en la parte central del swing.

### 3.4. Limitadores de velocidad (slew rate)

Para cada articulación se aplica:

\[
\theta_\text{permitida} \in [\theta_\text{prev} - \dot\theta_\max dt,\; \theta_\text{prev} + \dot\theta_\max dt]
\]

y se clipea el target dentro de ese rango. Esto evita:

- Cambios muy bruscos de ángulo.
- “Patadas” o impactos en la dinámica del robot.

### 3.5. Toe-off y pierna de apoyo

Durante swing de una pierna, la pierna de apoyo:

- Recibe un término de cadera (`stance_hip`) proporcional a `hip_target` para contrabalancear el movimiento.
- Puede recibir un **toe-off** (empuje plantar) en la parte final de la fase, dentro de una ventana \([\phi_1, \phi_2]\).

Para que haya toe-off se comprueban varias condiciones:

- `phi` dentro de la ventana grande (ej. 0.85–0.98).
- `toe_w` pasa de 0 a >0 (edge-trigger).
- Rodilla de la pierna en swing suficientemente flexionada.
- `roll` no demasiado grande.
- Temporizador de refractario `_toe_timer` en cero.

Ese toe-off es una pequeña contribución de ankle pitch que empuja el COM hacia adelante justo antes del cambio de soporte.

### 3.6. Gate por IMU

Se define un factor de amplitud `imu_amp_scale` que se reduce cuando el pitch IMU supera ciertos umbrales:

- Si |pitch| > IMU_GATE_ON → baja a ~0.7.
- Si |pitch| < IMU_GATE_OFF → vuelve hacia 1.0.

La transición se hace con una ecuación tipo:

\[
\text{scale}_\text{new} = 0.85\,\text{scale}_\text{old} + 0.15\,\text{target\_scale}
\]

para que sea suave.

---

## 4. Shaping de marcha y stride

### 4.1. Escalado por tipo de junta

En `apply_gait_offsets`, para cada joint se aplica un factor:

- Cadera pitch: `scale ≈ 1.25 * stride_gain`, con leves boosts extra si el robot está muy estable (pitch_f y gy pequeños).
- Rodilla: `scale = 0.80 + 0.40 * (stride_gain - 1.0)`.
- Tobillo: `scale = 0.95 + 0.35 * (stride_gain - 1.0)`.

La idea:

- A medida que se incrementa la zancada (`stride_gain`), se incrementan más las rodillas y tobillos, pero de manera controlada.

### 4.2. Matching de rodillas izquierda/derecha

Se define una función:

\[
\text{match}(a, b, k) =
\begin{cases}
\operatorname{sign}(a)\,k|b| & \text{si } |a| > k|b| \\
a & \text{en otro caso}
\end{cases}
\]

Se usa en dos etapas:

1. Según fase, para evitar que una rodilla tenga mucha más amplitud que la otra.
2. Según pierna en swing (`left_is_swing`), para que las rodillas no se desboquen desparejas.

Esto mantiene la marcha más simétrica y estable.

### 4.3. Cálculo de stride_gain

En `update_walk_outputs` se combinan:

- `STRIDE_BASE` (que tiende hacia un objetivo cuando el robot está estable).
- `soft_str` (tiempo desde el inicio de la marcha).
- Estabilidad en pitch y roll:

\[
stab_p = 1 - \frac{\min(0.12, |pitch_f|)}{0.12}
\]
\[
stab_r = 1 - \frac{\min(0.12, |roll_f|)}{0.12}
\]
\[
stability = stab_p \cdot stab_r
\]

Se define algo del estilo:

\[
stride\_gain = STRIDE\_BASE + 0.7 \, (soft\_str \cdot stability)
\]

y después se clipea a [STRIDE_MIN, STRIDE_MAX], con ajustes extra:

- Si el robot realmente se está yendo hacia atrás (`rel_dx` negativo), se reduce stride_gain.
- Si el torso se va demasiado hacia delante (`pitch_f` grande), se aplica un factor de damping y sesgos en tobillos/caderas.

### 4.4. Ganancia global del Walker

Se calcula una ganancia nominal:

\[
g_\text{nom} = gait\_gain \cdot soft\_gain \cdot trend\_brake \cdot micro
\]

donde:

- `soft_gain` viene del soft start de marcha.
- `trend_brake` reduce la ganancia si la tendencia de pitch (gy) indica caída.
- `micro` recorta los primeros pasos (warm up).

Después se impone un techo `g_cap` que depende:

- Del número de pasos,
- De la inclinación del torso,
- De si el último paso fue bueno o malo.

Finalmente:

\[
g_\text{walker} = \min(g_\text{cap}, \max(g_\text{min}, g_\text{nom}))
\]

y se aplica a través de `walker.set_global_gain(...)`.

---

## 5. IMU y estabilidad

### 5.1. Filtro exponencial para pitch_f y roll_f

Se usa:

\[
\alpha = e^{-dt/\tau}
\]

\[
pitch_f[n] = \alpha pitch_f[n-1] + (1-\alpha) pitch[n]
\]
\[
roll_f[n]  = \alpha roll_f[n-1]  + (1-\alpha) roll[n]
\]

Este filtro discreto de 1er orden reduce el ruido del IMU para decisiones de alto nivel (FSM, stride, etc.).

### 5.2. Acumulador de estabilidad

Se mantiene `stable_t`:

- Si \(|pitch_f| < P_STAB_LO\) y \(|roll_f| < R_STAB\):
  - \(stable_t = \min(STAB_T, stable_t + dt)\).
- Si se sale poco:
  - \(stable_t = \max(0, stable_t - 0.5 dt)\).
- Si se sale mucho:
  - \(stable_t = \max(0, stable_t - 2.0 dt)\).

Este contador se utiliza típicamente para decidir cuándo se puede pasar de un estado a otro (ej. de RECOVER a STAND, o autorizar WALK sólo si el robot ha estado un tiempo estable).

---

## 6. Evaluación de pasos (step_fsm)

Se calcula:

- `rel_dx` = avance relativo medido por GPS (con signo según la convención de Webots).
- `fwd_dx = -rel_dx` para tener positivo adelante.

Un paso se considera malo si:

- `pitch_f < step_pitch_back` (ej. −0.14 rad → demasiado inclinado hacia atrás), o
- `fwd_dx < step_dx_min` (no se avanzó suficiente hacia delante).

Se lleva un contador de pasos malos consecutivos `bad_run`; si supera `max_bad_steps`:

- FSM de pasos fuerza `state = "RECOVER"`.
- Resetea `recover_timer`.
- Pone en cero la ganancia de marcha (`gait_gain` y gain del Walker).
- Limpia offsets de tobillos/roll en `bal_prev`.

Interpretación: si el robot encadena varios pasos malos, mejor cortar la marcha y pasar a una fase de recuperación.

---

## 7. Límites y seguridad en articulaciones

### 7.1. Clamps por tipo de joint

Se definen límites simétricos ±lim según el tipo de articulación:

- Tobillos (`ankle_*`): `ank_lim`.
- Rodillas (`knee_pitch_*`): `knee_lim`.
- Caderas (`hip_*`): `hip_lim`.

Con `rclamp_with_flag`:

\[
x_\text{clamped} = \max(-lim, \min(lim, x))
\]

y si \(x_\text{clamped} \neq x\), se loguea y, si es tobillo, se marca `ankle_clamp_hit = True`.

Cuando hay una **pose estática** (levantar pierna), se amplían los límites
para permitir flexiones grandes en cadera y rodilla.

### 7.2. Detección de “clamp sostenido” en tobillos

Se maneja un contexto:

- `ankle_clamp_timer`: integra tiempo mientras haya clamps en tobillos.
- `ankle_clamp_hit`: se resetea en cada ciclo; se activa cuando un clamp afectó a un tobillo.

En `update_ankle_clamp_state`:

- Si `ankle_clamp_hit`:
  - `ankle_clamp_timer += dt`.
- Si no:
  - `ankle_clamp_timer = max(0, ankle_clamp_timer - DECAY*dt)`.

Si `state == "WALK"` y `ankle_clamp_timer >= CLAMP_TAU`:

- Se fuerza `state = "RECOVER"`.
- Se devuelve `recover_timer = 0.0`.

Esto significa que si el robot pasa demasiado tiempo golpeando el límite de tobillos mientras camina, la marcha se considera peligrosa y se activa el modo de recuperación.

---

## 8. Recuperación (RECOVER)

La función `recover_step` implementa una ley de control PD sobre pitch
con parámetros adaptativos según si el robot:

- Está cayendo de espaldas (“falling back”).
- Está cayendo hacia delante (“falling front”).
- Está cerca de vertical.

### 8.1. Ley PD adaptativa

Se fija un target:

\[
pitch_\text{target} = +0.02\;\text{rad}
\]

Se calcula:

\[
e = pitch_\text{target} - pitch_f
\]

Luego se distinguen tres regiones:

1. **Falling back**: \(pitch_f < BACK\_PITCH\) (e.g. −0.25 rad).
   - Se define:

     \[
     back = \max(0, -pitch_f)
     \]
     \[
     \alpha = \min(1, back / 0.40)
     \]

   - Se interpola \(K_p\) entre valores mínimos y máximos:

     \[
     K_p = K_{p,\min} + (K_{p,\max} - K_{p,\min})\,\alpha
     \]

   - Lo mismo para el límite \(u_\max\) (más margen cuanto más de espaldas).
   - Se usa un \(K_d\) moderado para amortiguar velocidad.

2. **Falling front**: \(pitch_f > FWD\_PITCH\) (e.g. +0.25 rad).
   - Similar lógica, pero con otros \(K_p, K_d, u_\max\).

3. **Zona cercana a vertical**:
   - Se usan valores suaves de \(K_p, K_d\) y límite \(u_\max\) pequeño.

En todos los casos:

\[
u_\text{raw} = K_p e - K_d \cdot gy
\]
\[
u = \operatorname{sat}_{[-u_\max, u_\max]}(u_\text{raw})
\]

Se loguean todos estos valores para analizar cómo responde el controlador.

### 8.2. Mapeo de `u` a joints

Se usan tres coeficientes:

- `k_hip`, `k_knee`, `k_ank`.

Y se aplican así:

- Caderas:

  \[
  \theta_{hip} \gets \theta_{hip} - k_{hip}\,u
  \]

  (recordando que hip pitch negativo = torso adelante).

- Rodillas:

  \[
  \theta_{knee} \gets \theta_{knee} - k_{knee}\,u
  \]

  De esta forma, cuando \(u > 0\) (back-fall), las rodillas **flexionan** (no se extienden), ayudando a absorber la caída.

- Tobillos:

  \[
  \theta_{ankle} \gets \theta_{ankle} + k_{ank}\,u
  \]

  Con saturaciones locales para no exceder rangos de seguridad.

Además, antes de esto, se hace un **mix hacia la pose base**:

\[
\theta_j \gets (1 - \alpha)\,\theta_j + \alpha\,\theta_{base,j}
\]

con \(\alpha\) pequeño al principio y algo mayor después de 0.2 s, para ir relajando progresivamente hacia la postura neutra sin generar choques bruscos.

---

## 9. Pose estática de levantar pierna

La FSM en `pose_fsm.py` tiene estados:

- `SHIFT`: desplazar COM hacia la pierna de apoyo.
- `UNLOAD`: descargar la pierna que se levantará.
- `LIFT`: flexionar cadera y rodilla de la pierna de swing.
- `HOLD`: mantener la pierna en el aire.

Cada fase tiene un tiempo (T_SHIFT, T_UNLOAD, T_LIFT) y se calcula un factor
de progreso:

\[
s = \min\left(1, \frac{t}{T}\right)
\]

que se usa para interpolar entre la pose base y la pose objetivo:

\[
\theta_j(t) = \theta_{base,j} + s\,(\theta_{objetivo,j} - \theta_{base,j})
\]

Además:

- Se aplican biases de roll y pitch en tobillos y caderas para asegurar que el COM
  cae sobre el pie de apoyo (tipo “pose de flamenco” estable).
- Se mantienen clamps y límites en las joints para no exceder rangos.

---

## 10. Telemetría y logging

### 10.1. RateLogger

`RateLogger` implementa:

- Niveles de log: `debug`, `info`, `warn`.
- Canales: `"gait"`, `"balance"`, `"gps"`, `"recover"`, etc.
- Rate limit: sólo deja pasar logs de cada canal cada `period = 1 / rate_hz`.

Se usa para que la consola no explote pero sí entregue información suficiente
para análisis offline.

### 10.2. Telemetría periódica

`telemetry.periodic_log(...)` imprime cada cierto tiempo:

- `state`, `gain` del Walker.
- `pitch`, `roll`, `steps`.
- Algunas articulaciones clave (rodilla, tobillo, cadera).
- Posición `x,z` por GPS.

El objetivo es poder correlacionar:

- Ganancias.
- Inclinaciones.
- Avance real.
- Actuaciones en joints.

y así afinar los parámetros del controlador.

---

## 11. Referencias conceptuales (no exhaustivo)

Las ideas de este controlador se apoyan en:

- Control PD/PID clásico, filtros de 1er orden y soft-start.
- Concepto de **estrategias ankle/hip** en control postural humano.
- Patrones de marcha bípeda:
  - Toe-off, heel-strike, mid-stance COM capture.
  - Uso de plantillas periódicas (gait generators) moduladas por feedback (IMU, contacto, avance).
- FSMs para manejar modos de operación: STAND, WALK, RECOVER, POSE.

Todo el diseño está orientado a tener un controlador:

- Lo suficientemente **simple** para tunear a mano.
- Lo bastante **modular** para extender (sensores de contacto, control ZMP, etc.).
- Y con una **base teórica clara** para que otra persona pueda comprender y seguir desarrollando el proyecto.
