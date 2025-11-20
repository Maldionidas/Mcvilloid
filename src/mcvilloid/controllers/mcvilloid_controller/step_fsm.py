# src/mcvilloid/controllers/mcvilloid_controller/step_fsm.py

def step_fsm_init():
    """
    Contexto de la FSM de pasos.
    """
    return {
        "index": 0,        # número de pasos evaluados
        "ok": True,        # si el último paso fue aceptable
        "bad_run": 0,      # pasos malos seguidos
        "prev_phase": 0.0, # fase anterior para detectar wrap
    }


def step_fsm_update(
    pitch_f,
    rel_dx,
    state,
    recover_timer,
    gait_gain,
    bal_prev,
    walker,
    LOG,
    ctx,
    step_pitch_back=-0.14,
    step_dx_min=-0.01,
    max_bad_steps=2,
):
    """
    Actualiza la FSM de pasos y, si hay demasiados pasos malos seguidos,
    fuerza transición a RECOVER.

    Devuelve: (state, recover_timer, gait_gain, ctx)
    """
    fwd_dx = -rel_dx
    # ¿paso malo?
    last_step_bad = (pitch_f < step_pitch_back) or (fwd_dx < step_dx_min)

    if last_step_bad:
        ctx["bad_run"] += 1
        ctx["ok"] = False
    else:
        ctx["bad_run"] = 0
        ctx["ok"] = True

    ctx["index"] += 1

    # Si hay muchos pasos malos seguidos → RECOVER
    if ctx["bad_run"] >= max_bad_steps:
        state = "RECOVER"
        recover_timer = 0.0
        gait_gain = 0.0
        walker.set_global_gain(0.0)
        LOG.info(
            "gait",
            f"[STEP_FSM] {ctx['bad_run']} pasos malos seguidos → RECOVER "
            f"(pitch_f={pitch_f:+.3f}, rel_dx={rel_dx:+.3f})",
        )
        bal_prev.clear()
        for j in (
            "j04_ankle_pitch_l",
            "j10_ankle_pitch_r",
            "j05_ankle_roll_l",
            "j11_ankle_roll_r",
        ):
            bal_prev[j] = 0.0

    return state, recover_timer, gait_gain, ctx
