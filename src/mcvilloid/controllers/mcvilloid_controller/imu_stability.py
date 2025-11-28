# src/mcvilloid/controllers/mcvilloid_controller/imu_stability.py
"""
Utilidades relacionadas con IMU y estabilidad para McVilloid.

Contiene dos piezas principales:

1) imu_update(...)
   - Lee los sensores IMU y gyro de Webots.
   - Calcula pitch/roll instantáneos.
   - Mantiene versiones filtradas (pitch_f, roll_f) con un filtro de 1er orden.
   - Devuelve también la componente de velocidad angular en pitch (gy).

2) update_stable_t(...)
   - Acumulador de “tiempo estable”:
       * integra mientras |pitch_f| y |roll_f| se mantengan en zona segura
       * descuenta (castiga) tiempo cuando el robot sale de esa zona
   - Este valor se usa como heurística para saber cuánto tiempo lleva estable
     antes de permitir WALK o ciertas transiciones.
"""

import math


def imu_update(
    imu,
    gyro,
    dt: float,
    TAU_IMU: float,
    pitch_f: float,
    roll_f: float,
    imu_f_ready: bool,
    LOG,
):
    """
    Lee IMU + gyro y actualiza los valores filtrados (pitch_f, roll_f).

    Parámetros
    ----------
    imu :
        Dispositivo IMU de Webots (puede ser None).
    gyro :
        Dispositivo Gyro de Webots (puede ser None).
    dt : float
        Paso de tiempo en segundos.
    TAU_IMU : float
        Constante de tiempo del filtro exponencial para pitch/roll.
    pitch_f : float
        Último pitch filtrado (rad).
    roll_f : float
        Último roll filtrado (rad).
    imu_f_ready : bool
        Indica si ya se inicializaron (True) o se debe hacer primer sample.
    LOG :
        Logger compatible con RateLogger (o None para no loguear).

    Devuelve
    --------
    pitch : float
        Pitch crudo de la IMU (rad).
    roll : float
        Roll crudo de la IMU (rad).
    gy : float
        Velocidad angular en pitch (rad/s) proveniente del gyro (0.0 si no hay).
    pitch_f : float
        Pitch filtrado (rad).
    roll_f : float
        Roll filtrado (rad).
    imu_f_ready : bool
        True tras el primer muestreo usado como inicialización del filtro.
    """
    pitch = 0.0
    roll = 0.0

    # --- Lectura IMU (roll, pitch, yaw) ---
    if imu:
        rpy = imu.getRollPitchYaw()
        roll, pitch = rpy[0], rpy[1]
        if LOG:
            LOG.debug("imu", f"r={roll:+.3f} p={pitch:+.3f}")

    # --- Lectura Gyro (velocidades angulares) ---
    gy = 0.0
    if gyro:
        # gx y gz no se usan aquí; sólo interesa la componente de pitch (gy)
        _, gy, _ = gyro.getValues()

    # --- Filtrado simple de 1er orden sobre pitch/roll ---
    # Este filtro sólo afecta a decisiones de alto nivel (guards),
    # no al lazo de balance PD.
    alpha = math.exp(-dt / TAU_IMU)

    if not imu_f_ready:
        # Primer sample: inicializa directamente con el valor actual
        pitch_f, roll_f, imu_f_ready = pitch, roll, True
    else:
        pitch_f = alpha * pitch_f + (1.0 - alpha) * pitch
        roll_f = alpha * roll_f + (1.0 - alpha) * roll

    return pitch, roll, gy, pitch_f, roll_f, imu_f_ready


def update_stable_t(
    state: str,
    stable_t: float,
    dt: float,
    pitch_f: float,
    roll_f: float,
    P_STAB_LO: float,
    R_STAB: float,
    STAB_T: float,
) -> float:
    """
    Actualiza el acumulador de estabilidad `stable_t`.

    Lógica
    ------
    - Sólo acumula cuando NO estamos en PRELEAN:
        * si |pitch_f| < P_STAB_LO y |roll_f| < R_STAB:
              stable_t aumenta hasta un máximo STAB_T
        * si estamos ligeramente fuera de rango:
              stable_t decrece lento (0.5 * dt)
        * si estamos claramente fuera de rango:
              stable_t decrece más rápido (2.0 * dt)
    - En PRELEAN se mantiene el valor (no acumula ni descuenta).

    Parámetros
    ----------
    state : str
        Estado global del gait FSM (STAND, PRELEAN, WALK, RECOVER, ...).
    stable_t : float
        Tiempo acumulado “estable” hasta el paso anterior.
    dt : float
        Paso de tiempo en segundos.
    pitch_f : float
        Pitch filtrado (rad).
    roll_f : float
        Roll filtrado (rad).
    P_STAB_LO : float
        Umbral bajo de pitch para considerar estable (rad).
    R_STAB : float
        Umbral de roll para considerar estable (rad).
    STAB_T : float
        Tiempo máximo de estabilidad acumulable.

    Devuelve
    --------
    stable_t : float
        Nuevo valor del tiempo acumulado de estabilidad.
    """
    if state != "PRELEAN":
        if abs(pitch_f) < P_STAB_LO and abs(roll_f) < R_STAB:
            # Zona claramente estable: integra hasta el máximo STAB_T
            stable_t = min(STAB_T, stable_t + dt)
        else:
            # Fuera de la zona, castiga más o menos según qué tanto nos salimos
            if (
                abs(pitch_f) < (P_STAB_LO + 0.03)
                and abs(roll_f) < (R_STAB + 0.03)
            ):
                # Un poco fuera: castigo suave
                stable_t = max(0.0, stable_t - 0.5 * dt)
            else:
                # Bastante fuera: castigo fuerte
                stable_t = max(0.0, stable_t - 2.0 * dt)

    return stable_t
