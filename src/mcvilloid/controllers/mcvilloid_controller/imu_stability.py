# src/mcvilloid/controllers/mcvilloid_controller/imu_stability.py
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

    Devuelve:
      pitch, roll, gy, pitch_f, roll_f, imu_f_ready
    """
    pitch = 0.0
    roll = 0.0
    if imu:
        rpy = imu.getRollPitchYaw()
        roll, pitch = rpy[0], rpy[1]
        if LOG:
            LOG.debug("imu", f"r={roll:+.3f} p={pitch:+.3f}")

    gy = 0.0
    if gyro:
        gx, gy, gz = gyro.getValues()

    # Filtrado simple 1er orden (no afecta a balance.py, solo decisiones)
    alpha = math.exp(-dt / TAU_IMU)
    if not imu_f_ready:
        pitch_f, roll_f, imu_f_ready = pitch, roll, True
    else:
        pitch_f = alpha * pitch_f + (1.0 - alpha) * pitch
        roll_f  = alpha * roll_f  + (1.0 - alpha) * roll

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
    Replica el acumulador de estabilidad que ya tenías:

    - Integra tiempo estable mientras |pitch_f| y |roll_f| sean pequeños
    - Castiga más fuerte cuando se sale mucho de rango
    """
    if state != "PRELEAN":
        if abs(pitch_f) < P_STAB_LO and abs(roll_f) < R_STAB:
            stable_t = min(STAB_T, stable_t + dt)
        else:
            if abs(pitch_f) < (P_STAB_LO + 0.03) and abs(roll_f) < (R_STAB + 0.03):
                stable_t = max(0.0, stable_t - 0.5 * dt)
            else:
                stable_t = max(0.0, stable_t - 2.0 * dt)
    return stable_t
