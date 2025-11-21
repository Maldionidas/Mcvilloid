# src/mcvilloid/controllers/mcvilloid_controller/gait_shaping.py
import math

try:
    from step_fsm import step_fsm_update
except ImportError:
    from .step_fsm import step_fsm_update


def apply_gait_offsets(
    gait_off,
    target,
    base_pose,
    stride_gain,
    pitch_f,
    gyro,
    gy,
    walker,
    left_is_swing,
):
    """
    Aplica los offsets de marcha (walker.step) a las juntas,
    reescalando por stride_gain y metiendo los match de rodillas.
    Modifica 'target' in-place.
    """
    if not gait_off:
        return

    for j, off in gait_off.items():
        if j not in target:
            continue

        # Escala por tipo de junta
        if "hip_pitch" in j:
            scale = 1.25 * stride_gain
            if (abs(pitch_f) < 0.18) and (not gyro or abs(gy) < 0.20):
                if abs(pitch_f) < 0.12 and (not gyro or abs(gy) < 0.12):
                    scale *= 1.08
                else:
                    scale *= 1.04
        elif "knee_pitch" in j:
            scale = 0.80 + 0.40 * (stride_gain - 1.0)
        elif "ankle_pitch" in j:
            scale = 0.95 + 0.35 * (stride_gain - 1.0)
        else:
            scale = 1.0

        target[j] += off * scale

    # --- Matching de rodillas izquierda/derecha ---
    phase = getattr(walker, "phase", 0.0)
    knee_L, knee_R = "j03_knee_pitch_l", "j09_knee_pitch_r"

    def _match(a, b, k=1.20):
        if abs(a) > k * abs(b):
            return math.copysign(k * abs(b), a)
        return a

    # Primera pasada: limitar asimetría global según fase
    if knee_L in target and knee_R in target:
        if phase < 0.5:
            target[knee_R] = _match(target[knee_R], target[knee_L], k=1.20)
        else:
            target[knee_L] = _match(target[knee_L], target[knee_R], k=1.20)

    # (Opcional) amplitud relativa respecto a base_pose (se mantiene aunque no se use)
    amp_L = target.get(knee_L, base_pose.get(knee_L, 0.0))
    amp_R = target.get(knee_R, base_pose.get(knee_R, 0.0))
    _ = (amp_L, amp_R)  # solo para evitar warnings de variable no usada

    # Segunda pasada: matching según pierna en swing
    if knee_L in target and knee_R in target:
        if left_is_swing:
            target[knee_L] = _match(target[knee_L], target[knee_R], k=1.20)
        else:
            target[knee_R] = _match(target[knee_R], target[knee_L], k=1.20)


#=============WALK==============#

def update_walk_outputs(
    dt,
    pitch,
    pitch_f,
    roll_f,
    gy,
    gyro,
    walker,
    walk_timer,
    gait_soft_t,
    GAIT_SOFT_S,
    STRIDE_BASE,
    STRIDE_MIN,
    STRIDE_MAX,
    STRIDE_WARM_S,
    WARM_STEPS,
    steps_taken,
    gait_gain,
    _dx_avg,
    base_pose,
    target,
    step_ctx,
    LOG,
    _tele_t,
    state,
    recover_timer,
    bal_prev,
):
    """
    Lógica completa de salidas cuando state == 'WALK':
      - soft start del gait
      - bias de tobillos en apoyo
      - conteo de pasos + step_fsm_update (bueno/malo)
      - cálculo de stride_gain
      - recortes por inestabilidad / pitch
      - toques de COM, toe-off, heel-strike, etc.
      - logging [TUNE]
    Devuelve:
      walk_timer, gait_soft_t, STRIDE_BASE, steps_taken,
      gait_gain, stride_gain, step_ctx, _tele_t,
      state, recover_timer, left_is_swing
    """
    # Avanza cronómetro de marcha
    walk_timer += dt

    # soft gain
    gait_soft_t = min(GAIT_SOFT_S, gait_soft_t + dt)
    soft_gain = min(1.0, max(0.0, gait_soft_t / GAIT_SOFT_S))

    phase = getattr(walker, "phase", 0.0)
    left_is_swing   = (phase >= 0.5)
    right_is_swing  = not left_is_swing
    left_is_support  = not left_is_swing
    right_is_support = not right_is_swing

    # (por ahora sin bias explícito de tobillo en apoyo)
    # SUPPORT_BIAS = -0.03
    # if left_is_support and "j04_ankle_pitch_l" in target:
    #     target["j04_ankle_pitch_l"] += SUPPORT_BIAS
    # if right_is_support and "j10_ankle_pitch_r" in target:
    #     target["j10_ankle_pitch_r"] += SUPPORT_BIAS

    # contar pasos + evento de nuevo paso (fase dio la vuelta)
    new_step = False
    if step_ctx["prev_phase"] > 0.90 and phase < 0.10:
        steps_taken += 1
        new_step = True
    step_ctx["prev_phase"] = phase

    if new_step:
        # medimos cómo estuvo el paso que ACABA de terminar
        rel_dx = _dx_avg * getattr(walker, "dir", 1.0)

        state, recover_timer, gait_gain, step_ctx = step_fsm_update(
            pitch_f,
            rel_dx,
            state,
            recover_timer,
            gait_gain,
            bal_prev,
            walker,
            LOG,
            step_ctx,
        )

    # flag de calidad del último paso
    step_ok = step_ctx.get("ok", True)

    # limitador primeros pasos
    if steps_taken < WARM_STEPS:
        micro = 0.55 + 0.15 * steps_taken
    else:
        micro = 1.0

    # ---------- estabilidad y boost de base ----------
    stable = (abs(pitch_f) < 0.18) and (not gyro or abs(gy) < 0.20)
    STRIDE_BASE_TARGET = 1.50  # target de zancada base
    if stable:
        STRIDE_BASE = 0.85 * STRIDE_BASE + 0.15 * min(STRIDE_MAX, STRIDE_BASE_TARGET)

    soft_str   = min(1.0, max(0.0, walk_timer / STRIDE_WARM_S))
    stab_p     = max(0.0, 1.0 - min(0.12, abs(pitch_f)) / 0.12)
    stab_r     = max(0.0, 1.0 - min(0.12, abs(roll_f))  / 0.12)
    stability  = stab_p * stab_r

    # stride extra algo agresivo
    stride_gain = STRIDE_BASE + 0.7 * (soft_str * stability)
    stride_gain = max(STRIDE_MIN, min(STRIDE_MAX, stride_gain))
    stride_gain *= micro

    # freno adicional por inestabilidad
    instab_pitch = max(0.0, -pitch)  # sólo cuando se va hacia atrás
    instab_roll  = max(0.0, abs(roll_f) - 0.06)

    k_pitch = 1.0 - min(0.5, (instab_pitch / 0.15) * 0.5)
    k_roll  = 1.0 - min(0.5, (instab_roll  / 0.14) * 0.5)
    k_instab = max(0.6, min(1.0, k_pitch * k_roll))
    stride_gain *= k_instab

    if not step_ok:
        # si el último paso fue malo, no dejes que la zancada crezca
        stride_gain *= 0.85

    rel_dx = _dx_avg * getattr(walker, "dir", 1.0)

    if walk_timer > 0.8 and rel_dx < -0.02:
        # Estamos "retrocediendo" cuando deberíamos avanzar:
        # baja zancada y también un poco la ganancia del gait.
        stride_gain *= 0.75
        gait_gain   = min(gait_gain, 0.20)

    # Anti-runaway si el torso se va al frente
    if pitch_f > 0.22:
        damp = max(0.60, 1.0 - (pitch_f - 0.22) / 0.20)
        stride_gain *= damp
        bias_df = +0.012 * (1.0 - damp)
        for ap in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
            if ap in target:
                target[ap] += bias_df
    if pitch_f > 0.26:
        for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
            if j in target:
                target[j] = max(-0.10, target[j])

    ppos = max(0.0, pitch_f)
    over_k = 0.0 if ppos < 0.22 else min(1.0, (ppos - 0.22) / (0.34 - 0.22))
    if over_k > 0.0:
        stride_gain *= (1.0 - 0.45 * over_k)
        HIP_BACK = +0.06 * over_k
        for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
            if hip in target:
                target[hip] += HIP_BACK
        if 0.0 <= phase < 0.5:
            ank_sup = "j10_ankle_pitch_r"
        else:
            ank_sup = "j04_ankle_pitch_l"
        if ank_sup in target:
            target[ank_sup] += +0.015 * over_k

    # Freno de stride cuando se va para atrás, pero más suave
    if (pitch_f < -0.04) or (gyro and gy < -0.16):
        negp = max(0.0, -pitch_f)  # solo cuando es negativo
        max_str = 1.0 - 1.2 * min(0.25, negp)
        max_str = max(0.65, max_str)
        stride_gain = min(stride_gain, max_str)

    HIP_MIDFF = min(0.045, 0.055 * stride_gain)
    for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
        target[hip] += HIP_MIDFF * 0.5

    pitch_rate = gy if gyro else 0.0

    # refuerzo de COM hacia delante en 8–18% de fase
    if 0.08 < phase < 0.18:
        right_is_support = (phase < 0.5)
        hip_sup = "j06_hip_pitch_r" if right_is_support else "j00_hip_pitch_l"
        ank_sup = "j10_ankle_pitch_r" if right_is_support else "j04_ankle_pitch_l"
        if hip_sup in target:
            target[hip_sup] += +0.012
        if ank_sup in target:
            target[ank_sup] += -0.018

    trend_brake = 1.0
    if (pitch < -0.03) or (pitch_rate < -0.12):
        trend_brake = 0.60
    if (pitch_f > 0.30) or (gyro and gy > 0.25):
        trend_brake *= 0.55

    g_nom = (gait_gain * soft_gain * trend_brake) * micro

    if steps_taken < 2:
        g_cap = 0.30
    elif steps_taken < 5:
        g_cap = 0.40
    else:
        g_cap = 0.60  # techo en marcha

    if not step_ok:
        # techo de ganancia más bajo si el último paso estuvo feo
        g_cap = min(g_cap, 0.30)

    # ----- recortes extra por inclinación del torso -----
    # Hacia delante
    if pitch_f > 0.26:
        g_cap = min(g_cap, 0.40)
    if pitch_f > 0.32:
        g_cap = min(g_cap, 0.32)

    if pitch_f < -0.10:
        g_cap = min(g_cap, 0.28)
    if pitch_f < -0.18:
        g_cap = min(g_cap, 0.20)

    # piso de ganancia un poco más bajo
    walker.set_global_gain(min(g_cap, max(0.18, g_nom)))

    # Telemetría debug (cada 0.5 s)
    _tele_t += dt
    if _tele_t >= 0.50:
        _tele_t = 0.0
        LOG.info(
            "gait",
            (
                f"[TUNE] phase={getattr(walker,'phase',0.0):.2f} "
                f"g={getattr(walker,'_global_gain',1.0):.2f} "
                f"soft={soft_gain:.2f} stride={stride_gain:.2f} "
                f"brake={trend_brake:.2f} "
                f"p_f={pitch_f:+.3f} r_f={roll_f:+.3f} "
                f"dx={_dx_avg:.4f} rel_dx={(_dx_avg*getattr(walker,'dir',1.0)):.4f} "
                f"strideBase={STRIDE_BASE:.2f}"
            ),
        )

    # Lifts de pierna en swing: sólo rodilla por ahora
    if abs(pitch_f) < 0.25 and gait_gain > 0.15:
        LIFT_KNEE_SW = 0.090
        LIFT_ANK_SW  = 0.0   # tobillo apagado

        # L swing
        if left_is_swing:
            if "j03_knee_pitch_l" in target:
                target["j03_knee_pitch_l"] += LIFT_KNEE_SW
            # if "j04_ankle_pitch_l" in target:
            #     target["j04_ankle_pitch_l"] += LIFT_ANK_SW

        # R swing
        if not left_is_swing:
            if "j09_knee_pitch_r" in target:
                target["j09_knee_pitch_r"] += LIFT_KNEE_SW
            # if "j10_ankle_pitch_r" in target:
            #     target["j10_ankle_pitch_r"] += LIFT_ANK_SW

    # Clamps locales
    if pitch < -0.08:
        BAL_PITCH_LIM_LOCAL = 0.06
        for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
            if j in target:
                ref = target[j]
                target[j] = max(-BAL_PITCH_LIM_LOCAL, min(BAL_PITCH_LIM_LOCAL, ref))

    if abs(roll_f) > 0.08:
        BAL_ROLL_LIM_LOCAL = 0.05
        for j in ("j05_ankle_roll_l", "j11_ankle_roll_r"):
            if j in target:
                ref = target[j]
                target[j] = max(-BAL_ROLL_LIM_LOCAL, min(BAL_ROLL_LIM_LOCAL, ref))

    # 4) Bias cuando se va de espaldas (pitch_f < 0):
    #    ahora SÓLO en tobillos, con signo contrario al caso de runaway hacia delante.
    negp = max(0.0, -pitch_f)
    if negp > 0.0:
        ANK_FF_K, ANK_FF_MAX = 0.5, 0.05
        ank_corr = +min(ANK_FF_MAX, ANK_FF_K * negp)  # ojo: signo POSITIVO
        for ap in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
            if ap in target:
                target[ap] += ank_corr
        # No tocamos caderas aquí para no pelear con el lazo de pitch principal

    # Swing retraction
    if (phase > 0.82) or (phase < 0.06):
        RETRACT = -0.025
        for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
            if hip in target:
                target[hip] += RETRACT

    # TOE-OFF con guard por pitch
    toe = 0.22
    W = 0.06
    if abs((phase - toe + 1.0) % 1.0) < W and pitch > -0.02:
        left_is_swing = (phase < 0.5)
        ank_sup = "j10_ankle_pitch_r" if left_is_swing else "j04_ankle_pitch_l"
        hip_sup = "j06_hip_pitch_r"   if left_is_swing else "j00_hip_pitch_l"

        if pitch_f < 0.18:
            if ank_sup in target:
                target[ank_sup] += -0.034
            if hip_sup in target:
                target[hip_sup] += +0.018
        else:
            if ank_sup in target:
                target[ank_sup] += -0.016
            if hip_sup in target:
                target[hip_sup] += +0.010
            target[ank_sup] += +0.014

    # Mid-stance COM capture
    if 0.45 < phase < 0.65:
        left_is_support = (phase < 0.5)
        ank_sup = "j10_ankle_pitch_r" if left_is_support else "j04_ankle_pitch_l"
        hip_sup = "j06_hip_pitch_r"   if left_is_support else "j00_hip_pitch_l"
        if ank_sup in target:
            target[ank_sup] += +0.016
        if pitch_f > 0.22 and hip_sup in target:
            target[hip_sup] += +0.012

    # HEEL-STRIKE amortiguado
    if (phase > 0.96) or (phase < 0.08):
        left_is_striking = (phase < 0.08)
        knee_strike = "j03_knee_pitch_l" if left_is_striking else "j09_knee_pitch_r"
        ank_strike  = "j04_ankle_pitch_l" if left_is_striking else "j10_ankle_pitch_r"
        if knee_strike in target:
            target[knee_strike] += +0.040
        if ank_strike  in target:
            target[ank_strike]  += +0.020
        ank_roll_strike = "j05_ankle_roll_l" if left_is_striking else "j11_ankle_roll_r"
        if ank_roll_strike in target:
            base = base_pose.get(ank_roll_strike, 0.0)
            target[ank_roll_strike] = 0.6 * target[ank_roll_strike] + 0.4 * base

    # Sesgo lateral en apoyo
    if (phase > 0.45) and (phase < 0.65):
        left_is_support = (phase < 0.5)
        hip_roll_sup = "j07_hip_roll_r" if left_is_support else "j01_hip_roll_l"
        if hip_roll_sup in target:
            target[hip_roll_sup] += 0.010

    # Return normal si no hubo trigger de RECOVER
    return (
        walk_timer,
        gait_soft_t,
        STRIDE_BASE,
        steps_taken,
        gait_gain,
        stride_gain,
        step_ctx,
        _tele_t,
        state,
        recover_timer,
        left_is_swing,
    )
