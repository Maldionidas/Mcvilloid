# src/mcvilloid/controllers/mcvilloid_controller/recover_fsm.py
"""
Rutina de recuperación (RECOVER) para McVilloid/HU.
--------------------------------------------------
Se llama en cada ciclo cuando el FSM global está en estado "RECOVER".

Objetivo:
- Llevar el torso de vuelta a casi vertical (ligeramente hacia adelante).
- Usar una ley tipo PD en pitch (con diferentes ganancias si cae hacia atrás
  o hacia adelante).
- Mapear la corrección a caderas, rodillas y tobillos de forma consistente
  con la convención del robot:
    * hip_pitch NEGATIVO  = torso hacia ADELANTE
    * ankle_pitch POSITIVO = toe-down (empuja COM hacia delante)
"""

from typing import Dict, Any


def recover_step(
    dt: float,
    pitch_f: float,
    roll_f: float,          # reservado por si en el futuro se usa en la ley
    gy_val: float,
    gyro_ok: bool,
    target: Dict[str, float],
    base_pose: Dict[str, float] | None,
    rec_pose: Dict[str, float],  # reservado / compatibilidad (por ahora no se usa)
    recover_timer: float,
    LOG: Any,
) -> float:
    """
    Un paso de la lógica de RECOVER.

    Parámetros
    ----------
    dt : float
        Paso de simulación (s).
    pitch_f : float
        Pitch filtrado de la IMU (rad).
    roll_f : float
        Roll filtrado (rad). Actualmente no se usa, pero se mantiene por compatibilidad.
    gy_val : float
        Velocidad angular en pitch (rad/s) proveniente del gyro.
    gyro_ok : bool
        Indica si el valor del gyro es fiable (sensor presente y habilitado).
    target : dict
        Diccionario {joint_name: ref_rad} que se está construyendo en el controller.
        recover_step modifica este diccionario IN-PLACE.
    base_pose : dict | None
        Pose neutra de referencia. Si no es None, se usa para relajar hacia la base.
    rec_pose : dict
        Estado interno reservado para futuras extensiones de RECOVER (por ahora no usado).
    recover_timer : float
        Tiempo acumulado en estado RECOVER (s).
    LOG : RateLogger-like
        Logger con método .info(ch, msg).

    Devuelve
    --------
    float
        Nuevo valor de recover_timer (tiempo acumulado en RECOVER).
    """
    recover_timer += dt

    # --- 0) thresholds sólo para CLASIFICAR (no disparan aquí) ---
    BACK_PITCH = -0.25  # ~-14°: claramente de espaldas
    FWD_PITCH  = +0.25  # ~+14°: claramente hacia delante

    falling_back  = (pitch_f < BACK_PITCH)
    falling_front = (pitch_f > FWD_PITCH)

    # --- 1) Relajar un poco hacia la pose base --------------------
    # Durante los primeros 0.2 s casi NO tiro hacia base, para no pelear
    # con la corrección fuerte inicial.
    if base_pose is not None:
        if recover_timer < 0.20:
            ALPHA = 0.02
        else:
            ALPHA = 0.08  # más suave que versiones anteriores

        for j, v_base in base_pose.items():
            if j in target:
                target[j] = (1.0 - ALPHA) * target[j] + ALPHA * v_base

    # Limitar ankle roll como antes (para no doblar raro lateral)
    for j in ("j05_ankle_roll_l", "j11_ankle_roll_r"):
        if j in target:
            target[j] = max(-0.04, min(0.04, target[j]))

    # --- 2) Ley PD sobre pitch, más agresiva cuando va de espaldas ---
    # Queremos que el torso termine casi vertical, un pelín hacia adelante.
    pitch_target = +0.02  # rad
    err = pitch_target - pitch_f

    if falling_back:
        # Caída hacia atrás
        back = max(0.0, -pitch_f)          # cuánto vamos de espaldas
        alpha = min(1.0, back / 0.40)      # 0..1 hasta ~0.40 rad

        # Kp y u_max suben con lo de espaldas que vamos
        Kp_min, Kp_max = 0.8, 1.6
        Kp = Kp_min + (Kp_max - Kp_min) * alpha

        Kd = 0.12

        u_raw = Kp * err
        if gyro_ok:
            u_raw -= Kd * gy_val

        U_MAX_MIN, U_MAX_MAX = 0.22, 0.50
        u_max = U_MAX_MIN + (U_MAX_MAX - U_MAX_MIN) * alpha

    elif falling_front:
        # Caída hacia adelante
        fwd = max(0.0, pitch_f)
        alpha = min(1.0, fwd / 0.35)

        Kp_min, Kp_max = 0.4, 1.2
        Kp = Kp_min + (Kp_max - Kp_min) * alpha
        Kd = 0.08

        u_raw = Kp * err
        if gyro_ok:
            u_raw -= Kd * gy_val

        u_max = 0.22  # menos agresivo hacia delante

    else:
        # Zona cercana a vertical: control suave
        Kp, Kd = 0.25, 0.06
        u_raw = Kp * err
        if gyro_ok:
            u_raw -= Kd * gy_val

        u_max = 0.16

    # Saturación final
    u = max(-u_max, min(u_max, u_raw))

    if LOG:
        LOG.info(
            "recover",
            (
                f"[REC] t={recover_timer:.2f}s pitch_f={pitch_f:+.3f} "
                f"err={err:+.3f} u={u:+.3f} u_max={u_max:.3f} "
                f"back={max(0.0,-pitch_f):+.3f} "
                f"front={max(0.0,pitch_f):+.3f}"
            ),
        )

    # --- 3) Mapear u a articulaciones --------------------------------
    # Convención:
    #   - hip_pitch NEGATIVO  = torso hacia ADELANTE
    #   - ankle_pitch POSITIVO = toe-down (empujar COM hacia delante)
    #
    # Para back-fall (u>0):
    #   - Cadera más NEGATIVA  (torso adelante).
    #   - Rodilla más FLEXIONADA.
    #   - Tobillo POSITIVO (toe-down) para empujar COM hacia delante.

    k_hip  = 0.45
    k_knee = 0.25
    k_ank  = 0.35

    # Caderas: empujan torso
    for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
        if hip in target:
            target[hip] += -k_hip * u
            # ampliar rango en RECOVER
            target[hip] = max(-0.60, min(0.30, target[hip]))

    # Rodillas: flexionan cuando u>0 (evita bloquear rodilla al recuperar)
    for knee in ("j03_knee_pitch_l", "j09_knee_pitch_r"):
        if knee in target:
            target[knee] += -k_knee * u
            target[knee] = max(0.15, min(0.55, target[knee]))

    # Tobillos: toe-down en back-fall (u>0)
    for ank in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
        if ank in target:
            target[ank] += +k_ank * u
            target[ank] = max(-0.18, min(0.18, target[ank]))

    return recover_timer
