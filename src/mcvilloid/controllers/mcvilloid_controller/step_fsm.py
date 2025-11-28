# src/mcvilloid/controllers/mcvilloid_controller/step_fsm.py
"""
FSM de evaluación de pasos (step-level watchdog).
-------------------------------------------------

Idea:
- Cada vez que termina un paso (detectado por wrap de fase en gait_shaping),
  se llama a step_fsm_update().
- Se evalúa si el paso fue "malo" según:
    * pitch_f muy hacia atrás (step_pitch_back)
    * avance relativo bajo (rel_dx / fwd_dx)
- Si se acumulan demasiados pasos malos seguidos (max_bad_steps),
  se fuerza una transición a RECOVER:
    * state = "RECOVER"
    * recover_timer = 0.0
    * gait_gain = 0.0
    * walker.set_global_gain(0.0)
    * se limpian offsets previos de balance en tobillos
"""


def step_fsm_init():
    """
    Inicializa el contexto de la FSM de pasos.

    Devuelve
    --------
    dict
        {
            "index": int       -> número de pasos evaluados hasta ahora,
            "ok": bool         -> True si el último paso fue aceptable,
            "bad_run": int     -> cuántos pasos malos consecutivos van,
            "prev_phase": float-> fase previa (0..1) para detectar wrap en gait
        }
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
    Actualiza la FSM de pasos con el resultado del último paso.

    Parámetros
    ----------
    pitch_f : float
        Pitch filtrado (rad) al final del paso. Negativo = se va hacia atrás.
    rel_dx : float
        Desplazamiento promedio relativo en X (m/s aprox) durante el paso,
        ya considerando la dirección del walker (se calcula en gait_shaping
        como _dx_avg * walker.dir).
    state : str
        Estado global de la FSM principal ("STAND", "WALK", "RECOVER", ...).
    recover_timer : float
        Timer actual de RECOVER (s).
    gait_gain : float
        Ganancia actual de marcha (0..1) que controla amplitud del Walker.
    bal_prev : dict
        Dict de offsets previos de balance en tobillos/roll, se limpia al
        forzar RECOVER.
    walker :
        Instancia del Walker, se usa para set_global_gain(0.0) si hay fallo.
    LOG :
        Logger con .info(ch, msg).
    ctx : dict
        Contexto de la FSM de pasos (ver step_fsm_init()).
    step_pitch_back : float
        Umbral de pitch_f hacia atrás para considerar el paso malo.
    step_dx_min : float
        Umbral de avance mínimo (en fwd_dx) para considerar el paso malo.
    max_bad_steps : int
        Pasos malos consecutivos necesarios para disparar RECOVER.

    Comportamiento
    --------------
    - Se considera que un paso es malo si:
        * pitch_f < step_pitch_back (muy de espaldas), o
        * fwd_dx < step_dx_min (poco avance hacia adelante).
    - Si hay max_bad_steps pasos malos seguidos:
        * state <- "RECOVER"
        * recover_timer <- 0.0
        * gait_gain <- 0.0
        * walker.set_global_gain(0.0)
        * se limpian los offsets de balance (bal_prev)
        * se loggea un mensaje [STEP_FSM]

    Devuelve
    --------
    (state, recover_timer, gait_gain, ctx)
    """
    # rel_dx viene ya "con signo de dirección", se define fwd_dx hacia adelante.
    fwd_dx = -rel_dx

    # ¿Paso malo?
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

        # Reset de offsets de balance en tobillos/roll
        bal_prev.clear()
        for j in (
            "j04_ankle_pitch_l",
            "j10_ankle_pitch_r",
            "j05_ankle_roll_l",
            "j11_ankle_roll_r",
        ):
            bal_prev[j] = 0.0

    return state, recover_timer, gait_gain, ctx
