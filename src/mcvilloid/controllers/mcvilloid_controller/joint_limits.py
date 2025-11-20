# joint_limits.py
import os
import sys

HERE = os.path.dirname(__file__)

# Import compatible con ejecución como script y como paquete
if __package__ in (None, ""):
    if HERE not in sys.path:
        sys.path.append(HERE)
    from safety_limits import rclamp_with_flag, update_ankle_clamp_state
else:
    from .safety_limits import rclamp_with_flag, update_ankle_clamp_state


def apply_joint_limits_and_clamps(
    target,
    pose,
    limits,
    clamp_ctx,
    state,
    CLAMP_TAU,
    dt,
    LOG,
    recover_timer,
):
    """
    Aplica límites de posición por tipo de joint y gestiona la lógica
    de 'clamp sostenido en tobillo -> RECOVER'.

    Devuelve:
      target (dict)      : referencias ya clamp-eadas
      state (str)        : posiblemente actualizado a "RECOVER"
      recover_timer (f)  : timer de RECOVER (puede reiniciarse)
    """
    # Límites base desde params.json
    ank_lim  = float(limits.get("ankle_pos", 0.22))
    hip_lim  = float(limits.get("hip_pos",   0.18))
    knee_lim = float(limits.get("knee_pos",  0.26))

    # Si estamos en modo POSE (levantar pierna), ampliamos límites
    if pose.get("on", False):
        hip_lim  = max(hip_lim,  1.20)
        knee_lim = max(knee_lim, 1.60)
        ank_lim  = max(ank_lim,  0.22)

    # Aplicar clamp por tipo de articulación
    for j, ref in list(target.items()):
        if ("ankle_pitch" in j) or ("ankle_roll" in j):
            target[j] = rclamp_with_flag(j, ref, ank_lim, clamp_ctx, LOG)
        elif "knee_pitch" in j:
            target[j] = rclamp_with_flag(j, ref, knee_lim, clamp_ctx, LOG)
        elif ("hip_pitch" in j) or ("hip_roll" in j):
            target[j] = rclamp_with_flag(j, ref, hip_lim, clamp_ctx, LOG)
        # otros joints los dejamos tal cual

    # Gestionar “clamp sostenido en tobillo → RECOVER”
    state, rec_reset = update_ankle_clamp_state(
        state,
        dt,
        CLAMP_TAU,
        clamp_ctx,
        LOG,
    )
    if rec_reset is not None:
        recover_timer = rec_reset

    return target, state, recover_timer
