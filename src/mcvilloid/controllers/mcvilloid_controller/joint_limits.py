# src/mcvilloid/controllers/mcvilloid_controller/joint_limits.py
"""
Límites y clamps de articulaciones para McVilloid.

Responsabilidades:
------------------
1) Aplicar límites de posición por tipo de articulación (tobillo, rodilla, cadera),
   usando los valores definidos en params["limits"].
2) Enviar cada joint a rclamp_with_flag(...) para:
     - recortar a ±lim
     - registrar en clamp_ctx si hay clamps repetidos
3) Usar update_ankle_clamp_state(...) para detectar “clamp sostenido en tobillos”
   y, si hace falta, forzar una transición a RECOVER y/o resetear recover_timer.

Se ejecuta al final del ciclo, cuando ya se han sumado:
   - neutral_pose
   - balance (BalanceController)
   - gait offsets (Walker + shaping)
   - POSE estática
"""

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
    target: dict,
    pose: dict,
    limits: dict,
    clamp_ctx: dict,
    state: str,
    CLAMP_TAU: float,
    dt: float,
    LOG,
    recover_timer: float,
):
    """
    Aplica límites de posición por tipo de joint y gestiona la lógica
    de “clamp sostenido en tobillo → RECOVER”.

    Parámetros
    ----------
    target : dict
        Referencias finales por joint (en rad). Se modifican in-place.
    pose : dict
        Estado actual de POSE estática (al menos keys: "on", "side", "phase"...).
    limits : dict
        Sub-árbol params["limits"] con claves opcionales:
          - "ankle_pos"
          - "hip_pos"
          - "knee_pos"
    clamp_ctx : dict
        Contexto interno para rclamp_with_flag / update_ankle_clamp_state
        (normalmente creado con clamp_ctx_init()).
    state : str
        Estado global del gait FSM (STAND, PRELEAN, WALK, RECOVER, ...).
    CLAMP_TAU : float
        Constante de tiempo para tratar “clamp sostenido”.
    dt : float
        Paso de tiempo en segundos.
    LOG :
        Logger tipo RateLogger (info/debug/warn) o compatible.
    recover_timer : float
        Tiempo acumulado en RECOVER (se puede resetear desde aquí).

    Devuelve
    --------
    target : dict
        Dict de referencias ya clamp-eadas (mismo objeto de entrada).
    state : str
        Estado posiblemente actualizado a "RECOVER" si hubo clamp grave en tobillos.
    recover_timer : float
        Timer de RECOVER, con posible reset si update_ankle_clamp_state lo indica.
    """
    # Límites base desde params.json (valores por defecto razonables si faltan)
    ank_lim  = float(limits.get("ankle_pos", 0.22))
    hip_lim  = float(limits.get("hip_pos",   0.18))
    knee_lim = float(limits.get("knee_pos",  0.26))

    # Si estamos en modo POSE (levantar pierna), ampliamos límites
    if pose.get("on", False):
        # Para POSE no queremos que los límites “normales” corten el movimiento.
        hip_lim  = max(hip_lim,  1.20)
        knee_lim = max(knee_lim, 1.60)
        ank_lim  = max(ank_lim,  0.22)

    # --- Aplicar clamp por tipo de articulación ---
    for j, ref in list(target.items()):
        if ("ankle_pitch" in j) or ("ankle_roll" in j):
            target[j] = rclamp_with_flag(j, ref, ank_lim, clamp_ctx, LOG)
        elif "knee_pitch" in j:
            target[j] = rclamp_with_flag(j, ref, knee_lim, clamp_ctx, LOG)
        elif ("hip_pitch" in j) or ("hip_roll" in j):
            target[j] = rclamp_with_flag(j, ref, hip_lim, clamp_ctx, LOG)
        # otros joints los dejamos tal cual

    # --- Gestionar “clamp sostenido en tobillo → RECOVER” ---
    state, rec_reset = update_ankle_clamp_state(
        state=state,
        dt=dt,
        CLAMP_TAU=CLAMP_TAU,
        clamp_ctx=clamp_ctx,
        LOG=LOG,
    )

    if rec_reset is not None:
        # update_ankle_clamp_state pide reiniciar el timer de RECOVER
        recover_timer = rec_reset

    return target, state, recover_timer
