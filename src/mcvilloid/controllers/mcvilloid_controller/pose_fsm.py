# src/mcvilloid/controllers/mcvilloid_controller/pose_fsm.py
"""
FSM para la pose estática de levantar pierna (teclas F / G / H).

Propósito
---------
Gestiona una pose “cuasi-estática” de levantar una pierna mientras la otra
mantiene el soporte, separada de la marcha cíclica del Walker.

Estados (pose["phase"])
-----------------------
- IDLE   : sin pose activa.
- SHIFT  : traslada el peso lateralmente y adelanta ligeramente el torso.
- UNLOAD : descarga la pierna que se va a levantar (inicio de dorsiflexión).
- LIFT   : flexiona cadera y rodilla de la pierna en swing y aumenta dorsiflexión.
- HOLD   : mantiene la pierna en el aire en una postura fija.

Estructura de `pose`
--------------------
pose = {
    "on"    : bool,    # True si la pose está activa
    "side"  : "L"|"R", # pierna en swing (L = izquierda, R = derecha)
    "phase" : str,     # IDLE / SHIFT / UNLOAD / LIFT / HOLD
    "t"     : float,   # tiempo acumulado en la fase actual (s)
}

API pública
-----------
- pose_init()  -> dict           : crea el diccionario de estado.
- pose_start(pose, side, LOG)    : arranca la pose en lado L/R.
- pose_cancel(pose, LOG)        : cancela y vuelve a IDLE.
- update_pose(dt, pose, target, base_pose, LOG):
    actualiza la pose y modifica `target` in-place.
"""


def pose_init() -> dict:
    """Crea el diccionario de estado inicial de pose."""
    return {"on": False, "side": "L", "phase": "IDLE", "t": 0.0}


def pose_start(pose: dict, side: str, LOG) -> None:
    """
    Inicia la pose levantando pierna IZQ ('L') o DER ('R').

    No enciende ni apaga el Walker: eso se hace en el controller
    (por ejemplo, al presionar F/G se suele hacer walker.toggle(False)).
    """
    pose.update({"on": True, "side": side, "phase": "SHIFT", "t": 0.0})
    lado = "IZQ" if side == "L" else "DER"
    LOG.info("gait", f"[POSE] Levantar {lado}: SHIFT")


def pose_cancel(pose: dict, LOG) -> None:
    """Cancela la pose y vuelve a IDLE."""
    pose.update({"on": False, "phase": "IDLE", "t": 0.0})
    LOG.info("gait", "[POSE] cancelada")


def update_pose(
    dt: float,
    pose: dict,
    target: dict,
    base_pose: dict,
    LOG,
) -> None:
    """
    Actualiza la pose estática de levantar pierna (SHIFT → UNLOAD → LIFT → HOLD).

    Parámetros
    ----------
    dt : float
        Paso de tiempo en segundos.
    pose : dict
        Estado de la FSM de pose (modifica "phase" y "t").
    target : dict
        Referencias de joints objetivo (en rad). Se modifica in-place.
    base_pose : dict
        Pose neutra de referencia (neutral_pose) para sumar offsets limpios.
    LOG :
        Logger tipo RateLogger o compatible.
    """
    if not pose.get("on", False):
        return

    pose["t"] += dt

    left = (pose["side"] == "L")
    sup   = "R" if left else "L"   # pierna de soporte
    swing = "L" if left else "R"   # pierna que se levanta

    # Nombres de joints según lado
    hip_sw   = "j00_hip_pitch_l"   if swing == "L" else "j06_hip_pitch_r"
    knee_sw  = "j03_knee_pitch_l"  if swing == "L" else "j09_knee_pitch_r"
    ankP_sup = "j10_ankle_pitch_r" if sup   == "R" else "j04_ankle_pitch_l"
    ankR_sup = "j11_ankle_roll_r"  if sup   == "R" else "j05_ankle_roll_l"
    ankP_sw  = "j04_ankle_pitch_l" if swing == "L" else "j10_ankle_pitch_r"
    ankR_sw  = "j05_ankle_roll_l"  if swing == "L" else "j11_ankle_roll_r"

    # Constantes de tiempo y amplitud para las fases
    T_SHIFT   = 0.35
    T_UNLOAD  = 0.20
    T_LIFT    = 0.35
    HIP_FLEX  = -1.60
    KNEE_FLEX = +1.0
    ANK_DORSI = +0.12
    TORSO_FWD = -0.05
    ROLL_BIAS = +0.12 if sup == "R" else -0.12
    ANK_SUP_P = -0.04
    ANK_SUP_R = ROLL_BIAS * 0.6

    phase_p = pose["phase"]

    # ----------------- SHIFT: mover COM y cargar pierna de soporte -----------------
    if phase_p == "SHIFT":
        s = min(1.0, pose["t"] / T_SHIFT)

        # torso ligeramente hacia adelante
        for hp in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
            if hp in target:
                target[hp] += TORSO_FWD * s

        # inclinar roll hacia la pierna de soporte y ajustar tobillo de soporte
        if ankR_sup in target:
            target[ankR_sup] += ANK_SUP_R * s
        if ankR_sw in target:
            target[ankR_sw] -= ANK_SUP_R * s
        if ankP_sup in target:
            target[ankP_sup] += ANK_SUP_P * s

        if pose["t"] >= T_SHIFT:
            pose.update({"phase": "UNLOAD", "t": 0.0})
            LOG.info("gait", "[POSE] UNLOAD")

    # ----------------- UNLOAD: descargar pierna que se va a levantar -----------------
    elif phase_p == "UNLOAD":
        s = min(1.0, pose["t"] / T_UNLOAD)

        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI * 0.2 * s
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P

        if pose["t"] >= T_UNLOAD:
            pose.update({"phase": "LIFT", "t": 0.0})
            LOG.info("gait", "[POSE] LIFT")

    # ----------------- LIFT: levantar pierna swing -----------------
    elif phase_p == "LIFT":
        s = min(1.0, pose["t"] / T_LIFT)

        # mantener soporte preparado
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P
        if ankR_sup in target:
            target[ankR_sup] = base_pose.get(ankR_sup, 0.0) + ANK_SUP_R
        if ankR_sw in target:
            target[ankR_sw] = base_pose.get(ankR_sw, 0.0) - ANK_SUP_R

        # levantar la pierna swing: cadera + rodilla + dorsiflexión
        if hip_sw in target:
            target[hip_sw] = base_pose.get(hip_sw, 0.0) + HIP_FLEX * s
        if knee_sw in target:
            target[knee_sw] = base_pose.get(knee_sw, 0.0) + KNEE_FLEX * s
        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI * s

        if pose["t"] >= T_LIFT:
            pose.update({"phase": "HOLD", "t": 0.0})
            LOG.info("gait", "[POSE] HOLD")

    # ----------------- HOLD: mantener pierna en el aire -----------------
    elif phase_p == "HOLD":
        # pierna de soporte estable
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P
        if ankR_sup in target:
            target[ankR_sup] = base_pose.get(ankR_sup, 0.0) + ANK_SUP_R
        if ankR_sw in target:
            target[ankR_sw] = base_pose.get(ankR_sw, 0.0) - ANK_SUP_R

        # pierna levantada congelada en la pose final
        if hip_sw in target:
            target[hip_sw] = base_pose.get(hip_sw, 0.0) + HIP_FLEX
        if knee_sw in target:
            target[knee_sw] = base_pose.get(knee_sw, 0.0) + KNEE_FLEX
        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI

    # Otros valores de phase (por seguridad, aunque no deberían aparecer)
    else:
        LOG.warn("gait", f"[POSE] phase desconocida='{phase_p}', cancelando pose")
        pose_cancel(pose, LOG)
