# pose_fsm.py
"""
FSM para la pose estática de levantar pierna (F / G / H).
"""

def pose_init():
    """Crea el diccionario de estado de pose."""
    return {"on": False, "side": "L", "phase": "IDLE", "t": 0.0}


def pose_start(pose, side, LOG):
    """Inicia la pose levantando pierna IZQ ('L') o DER ('R')."""
    pose.update({"on": True, "side": side, "phase": "SHIFT", "t": 0.0})
    lado = "IZQ" if side == "L" else "DER"
    LOG.info("gait", f"[POSE] Levantar {lado}: SHIFT")


def pose_cancel(pose, LOG):
    """Cancela la pose y vuelve a IDLE."""
    pose.update({"on": False, "phase": "IDLE", "t": 0.0})
    LOG.info("gait", "[POSE] cancelada")


def update_pose(dt, pose, target, base_pose, LOG):
    """Actualiza la pose estática de levantar pierna (SHIFT → UNLOAD → LIFT → HOLD)."""
    if not pose["on"]:
        return

    pose["t"] += dt
    left  = (pose["side"] == "L")
    sup   = "R" if left else "L"
    swing = "L" if left else "R"

    hip_sup   = "j06_hip_pitch_r" if sup == "R" else "j00_hip_pitch_l"
    hip_sw    = "j00_hip_pitch_l" if swing == "L" else "j06_hip_pitch_r"
    knee_sw   = "j03_knee_pitch_l" if swing == "L" else "j09_knee_pitch_r"
    ankP_sup  = "j10_ankle_pitch_r" if sup == "R" else "j04_ankle_pitch_l"
    ankR_sup  = "j11_ankle_roll_r"  if sup == "R" else "j05_ankle_roll_l"
    ankP_sw   = "j04_ankle_pitch_l" if swing == "L" else "j10_ankle_pitch_r"
    ankR_sw   = "j05_ankle_roll_l"  if swing == "L" else "j11_ankle_roll_r"

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

    if phase_p == "SHIFT":
        s = min(1.0, pose["t"] / T_SHIFT)
        for hp in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
            if hp in target:
                target[hp] += TORSO_FWD * s
        if ankR_sup in target:
            target[ankR_sup] += ANK_SUP_R * s
        if ankR_sw  in target:
            target[ankR_sw]  -= ANK_SUP_R * s
        if ankP_sup in target:
            target[ankP_sup] += ANK_SUP_P * s

        if pose["t"] >= T_SHIFT:
            pose.update({"phase": "UNLOAD", "t": 0.0})
            LOG.info("gait", "[POSE] UNLOAD")

    elif phase_p == "UNLOAD":
        s = min(1.0, pose["t"] / T_UNLOAD)
        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI * 0.2 * s
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P
        if pose["t"] >= T_UNLOAD:
            pose.update({"phase": "LIFT", "t": 0.0})
            LOG.info("gait", "[POSE] LIFT")

    elif phase_p == "LIFT":
        s = min(1.0, pose["t"] / T_LIFT)
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P
        if ankR_sup in target:
            target[ankR_sup] = base_pose.get(ankR_sup, 0.0) + ANK_SUP_R
        if ankR_sw  in target:
            target[ankR_sw]  = base_pose.get(ankR_sw, 0.0) - ANK_SUP_R

        if hip_sw in target:
            target[hip_sw]  = base_pose.get(hip_sw, 0.0) + HIP_FLEX  * s
        if knee_sw in target:
            target[knee_sw] = base_pose.get(knee_sw, 0.0) + KNEE_FLEX * s
        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI * s

        if pose["t"] >= T_LIFT:
            pose.update({"phase": "HOLD", "t": 0.0})
            LOG.info("gait", "[POSE] HOLD")

    elif phase_p == "HOLD":
        if ankP_sup in target:
            target[ankP_sup] = base_pose.get(ankP_sup, 0.0) + ANK_SUP_P
        if ankR_sup in target:
            target[ankR_sup] = base_pose.get(ankR_sup, 0.0) + ANK_SUP_R
        if ankR_sw  in target:
            target[ankR_sw]  = base_pose.get(ankR_sw, 0.0) - ANK_SUP_R
        if hip_sw  in target:
            target[hip_sw]  = base_pose.get(hip_sw, 0.0) + HIP_FLEX
        if knee_sw in target:
            target[knee_sw] = base_pose.get(knee_sw, 0.0) + KNEE_FLEX
        if ankP_sw in target:
            target[ankP_sw] = base_pose.get(ankP_sw, 0.0) + ANK_DORSI
