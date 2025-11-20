# src/mcvilloid/controllers/mcvilloid_controller/mcvilloid_controller.py
import os, sys, json, math, time
from controller import Robot

HERE = os.path.dirname(__file__)

# --- Imports compatibles con ejecución como script en Webots y como paquete ---
if __package__ in (None, ""):
    if HERE not in sys.path:
        sys.path.append(HERE)
    from balance.balance import BalanceController
    from movement.walking import Walker
    from pose_fsm import pose_init, pose_start, pose_cancel, update_pose
    from recover_fsm import recover_step
    from controls import handle_keyboard
    from step_fsm import step_fsm_update, step_fsm_init
    from rate_logger import RateLogger
    from telemetry import periodic_log
    from gait_shaping import apply_gait_offsets, update_walk_outputs
    from safety_limits import (clamp_ctx_init,
        rclamp_with_flag,
        update_ankle_clamp_state,)
    from imu_stability import imu_update, update_stable_t
    from joint_limits import apply_joint_limits_and_clamps
    from balance_shaping import compute_balance_offsets

else:
    from .balance.balance import BalanceController
    from .movement.walking import Walker
    from .pose_fsm import pose_init, pose_start, pose_cancel, update_pose
    from .recover_fsm import recover_step
    from .controls import handle_keyboard
    from .step_fsm import step_fsm_update, step_fsm_init
    from .rate_logger import RateLogger
    from .telemetry import periodic_log
    from .gait_shaping import apply_gait_offsets, update_walk_outputs
    from .safety_limits import (clamp_ctx_init,
        rclamp_with_flag,
        update_ankle_clamp_state,)
    from .imu_stability import imu_update, update_stable_t
    from .joint_limits import apply_joint_limits_and_clamps
    from .balance_shaping import compute_balance_offsets

TIME_STEP = 16  # ms

def load_params():
    with open(os.path.join(HERE, "params.json"), "r") as f:
        return json.load(f)

def run():
    robot = Robot()
    dt = TIME_STEP / 1000.0

    # --- Config y logger ---
    params   = load_params()
    limits   = params.get("limits", {})
    log_cfg  = params.get("logging", {})
    LOG      = RateLogger(log_cfg, time.monotonic)

    LOG.info("boot", "mcvilloid_controller: init")
    LOG.debug("boot", f"params keys: {list(params.keys())}")

    # --- Teclado ---
    kb = robot.getKeyboard()
    kb.enable(TIME_STEP)

    # --- Dispositivos ---
    imu_name = params.get("imu_name", "imu_sensor")
    imu = robot.getDevice(imu_name)
    if imu:
        imu.enable(TIME_STEP)
        LOG.info("boot", f"IMU '{imu_name}' OK")
    else:
        LOG.warn("boot", f"IMU '{imu_name}' NOT FOUND")

    gyro = robot.getDevice(params.get("gyro_name", "gyro"))
    if gyro:
        gyro.enable(TIME_STEP)

    gps = robot.getDevice('gps')
    if gps:
        gps.enable(TIME_STEP)

    # Vars de tracking de marcha / GPS (deben existir aunque no haya gps)
    _dx_acc = 0.0
    _x_prev = None
    _dx_avg = 0.0
    _walk_time_since_on = 0.0
    # --- Motores (desde params.json) ---
    motor_names = params.get("motors", [])
    motors = {}
    for n in motor_names:
        dev = robot.getDevice(n)
        if dev:
            motors[n] = dev
        else:
            LOG.warn("boot", f"Motor '{n}' NOT FOUND")
    LOG.info("boot", f"Motores OK: {sorted(motors.keys())}")

    # Límites de velocidad por tipo (valores razonables)
    ank_vel = float(limits.get("ankle_vel", 1.0))
    hip_vel = float(limits.get("hip_vel",   1.8))
    for name, m in motors.items():
        if "knee_" in name:
            m.setVelocity(3.5)
        elif "ankle_" in name:
            m.setVelocity(ank_vel)
        elif any(s in name for s in ["hip_", "yaw_"]):
            m.setVelocity(hip_vel)
        else:
            m.setVelocity(2.0)

    # Postura neutra
    neutral_cfg = (params.get("neutral_pose") or {})
    base_pose = {name: float(neutral_cfg.get(name, 0.0)) for name in motors.keys()}

    # Anti-trasera: leve plantar de base en ambos tobillos pitch
    #for ank in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
    #    if ank in base_pose:
    #        base_pose[ank] += -0.02  # -0.02 rad ≈ -1.1°

    for n, m in motors.items():
        m.setPosition(base_pose[n])

    # --- Controladores ---
    bal    = BalanceController(params, logger=lambda *a: LOG.info("balance", " ".join(map(str, a))))
    walker = Walker(params, limits,    logger=lambda *a: LOG.info("gait",    " ".join(map(str, a))))

    # Arranque del gait
    walker.toggle(bool(params.get("gait", {}).get("enabled_on_start", False)))
    LOG.info("gait", f"Walker: {'ON' if getattr(walker, 'on', False) else 'OFF'}")

    # Estado/diagnóstico
    step_count    = 0
    t             = 0.0
    last_log_t    = 0.0
    sample_joints = ["j03_knee_pitch_l", "j09_knee_pitch_r", "j04_ankle_pitch_l"]

    # ======================== FSM Gait ========================
    state = "STAND"
    gait_enable = False
    walker_last_on = False

    prelean_timer = 0.0
    recover_timer = 0.0
    PRELEAN_TIME = 0.30
    PRELEAN_HIP  = 0.040  # rampa hacia flexión (torso un poco adelante)
    gait_soft_t = 0.0
    GAIT_SOFT_S = float(params.get("gait", {}).get("soft_start_s", 2.0))
    # --- Escalado de zancada (ajustada) ---
    STRIDE_BASE   = float(params.get("gait", {}).get("stride_base", 1.0))
    STRIDE_MAX    = float(params.get("gait", {}).get("stride_max",  1.55))  # ↑ techo
    STRIDE_MIN    = 0.90
    STRIDE_WARM_S = 1.40  # s para “soltar” la zancada al arrancar WALK
    # Umbrales IMU (para guards/decisiones)
    T_ENTER = 0.10
    T_EXIT  = 0.08
    T_FALL  = 0.34
    T_WARN  = 0.18
    CLAMP_TAU    = 0.25
    # --- helper: clamp con log ---
    clamp_ctx = clamp_ctx_init()

    # ---- Balance: límites y rate-limit (pitch vs roll) ----
    BAL_PITCH_LIM = 0.09
    bal_prev = {}
    # --- IMU smoothing (para decisiones/guards) ---
    TAU_IMU = 0.12  # s
    pitch_f = 0.0
    roll_f  = 0.0
    imu_f_ready = False
    # --- Histeresis de estabilidad para autorizar WALK (ajustada) ---
    stable_t = 0.0
    STAB_T    = 0.28
    P_STAB_LO = 0.010
    R_STAB    = 0.08
    # --- Cronómetro y warm-up de pasos ---
    walk_timer = 0.0
    steps_taken = 0
    WARM_STEPS = 3  # limitar zancada y ganancia en los 3 primeros pasos

    # === FSM simple por paso (bueno/malo) ===
    step_ctx = step_fsm_init()

    # ======================== FSM POSE (levantar pierna estática) ========================
    pose = pose_init()

    # --- Estado para RECOVER (hold y detección de mejora) ---
    recover_hold_ok_t = 0.0

    # ---------------------- Bucle principal ----------------------
    _rec_pose = {}
    _tele_t = 0.0

    while robot.step(TIME_STEP) != -1:
        step_count += 1
        t += dt

        if state == "WALK":
            _walk_time_since_on += dt
        else:
            _walk_time_since_on = 0.0

        target = base_pose.copy()
        gait_gain = 1.0
        stride_gain = 1.0
        left_is_swing = False

        # --- Teclas (delegado a controls.py) ---
        gait_enable, STRIDE_BASE = handle_keyboard(
            kb,
            walker,
            pose,
            STRIDE_BASE,
            STRIDE_MIN,
            STRIDE_MAX,
            gait_enable,
            LOG,
        )

        # 1) IMU + filtrado + estabilidad (delegado a imu_stability.py)
        pitch, roll, gy, pitch_f, roll_f, imu_f_ready = imu_update(imu,
            gyro,
            dt,
            TAU_IMU,
            pitch_f,
            roll_f,
            imu_f_ready,
            LOG,)

        stable_t = update_stable_t(state,
            stable_t,
            dt,
            pitch_f,
            roll_f,
            P_STAB_LO,
            R_STAB,
            STAB_T,)


        # 2) Balance (delegado a balance_shaping.py)
        bal_post = compute_balance_offsets(dt,
            bal,
            pitch,
            roll,
            pitch_f,
            BAL_PITCH_LIM,
            state,
            LOG,)

        # --- Amordazar balance en RECOVER y permitir solo dorsiflexión en tobillo ---
        if state == "RECOVER" and bal_post:
            for j in list(bal_post.keys()):
                bal_post[j] *= 0.45
            for ap in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
                if ap in bal_post:
                    lim_local = 0.08 if pitch_f > 0.16 else 0.04
                    bal_post[ap] = max(0.0, min(lim_local, bal_post[ap]))

        # aplicar balance con escala menor en STAND
        bal_scale_state = 0.45 if state == "STAND" else 1.0
        for j, off in bal_post.items():
            target[j] += off * bal_scale_state

        if bal_post and (("j04_ankle_pitch_l" in bal_post) or ("j10_ankle_pitch_r" in bal_post)
                         or ("j05_ankle_roll_l" in bal_post) or ("j11_ankle_roll_r" in bal_post)):
            ap_l = bal_post.get("j04_ankle_pitch_l", 0.0)
            ap_r = bal_post.get("j10_ankle_pitch_r", 0.0)
            ar_l = bal_post.get("j05_ankle_roll_l",  0.0)
            ar_r = bal_post.get("j11_ankle_roll_r",  0.0)
            LOG.info("balance", f"Δankle L(p:{ap_l:+.3f}, r:{ar_l:+.3f}) R(p:{ap_r:+.3f}, r:{ar_r:+.3f})")

        # 3) Guards de amplitud (gait_gain) por IMU con histéresis y piso
        p_abs = abs(pitch_f)
        r_abs = abs(roll_f)

        P_SOFT = 0.04
        P_HARD = 0.09
        GAIN_FLOOR = 0.05

        if p_abs <= P_SOFT:
            g_pitch = 1.0
        elif p_abs >= P_HARD:
            g_pitch = GAIN_FLOOR
        else:
            t_norm = (p_abs - P_SOFT) / (P_HARD - P_SOFT)
            g_pitch = max(GAIN_FLOOR, (1.0 - t_norm) ** 2)

        if pitch_f < 0.0:
            g_pitch = max(GAIN_FLOOR, g_pitch * (1.0 - 5.0 * max(0.0, -pitch_f)))

        if r_abs <= 0.06:
            g_roll = 1.0
        elif r_abs >= 0.18:
            g_roll = GAIN_FLOOR
        else:
            t_norm = (r_abs - 0.06) / (0.18 - 0.06)
            g_roll = max(GAIN_FLOOR, (1.0 - t_norm) ** 2)
        g_roll *= max(GAIN_FLOOR, 1.0 - 2.0 * max(0.0, r_abs - 0.06))

        gait_gain = min(g_pitch, g_roll)

        # 4) FSM Gait principal
        can_walk     = abs(pitch_f) < T_ENTER
        near_fall = (
            (abs(pitch_f) > T_FALL) or
            (abs(roll_f)  > 0.32)   or
            (abs(pitch_f) > max(T_WARN, 0.20) and gait_gain < 0.20)
        )

        if state == "STAND":
            walk_timer = 0.0
            if near_fall:
                state = "RECOVER"
                recover_timer = 0.0
                bal_prev.clear()
                for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r", "j05_ankle_roll_l", "j11_ankle_roll_r"):
                    bal_prev[j] = 0.0
            elif gait_enable and can_walk and (abs(roll_f) < R_STAB) and not pose["on"]:
                state = "PRELEAN"
                prelean_timer = 0.0

        elif state == "WALK":
            SAFE_GRACE_S = 0.35

            # --- NUEVO: detección explícita de caída hacia atrás ---
            gy_val = gy if gyro else 0.0
            BACK_TRIG = -0.11   # ~ -6°
            GY_TRIG   = 0.40    # rad/s aprox

            back_fall_now = (pitch_f < BACK_TRIG and gy_val < -GY_TRIG)

            if back_fall_now:
                LOG.info("recover", f"[TRIG_BACK_WALK] pitch_f={pitch_f:+.3f} gy={gy_val:+.3f}")
                state = "RECOVER"
                recover_timer = 0.0
                bal_prev.clear()
                for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r",
                          "j05_ankle_roll_l", "j11_ankle_roll_r"):
                    bal_prev[j] = 0.0

            else:
                # Sólo detectamos near_fall cuando se va de cara o por roll
                T_FALL_EFF = max(T_FALL, 0.40)
                T_WARN_EFF = max(T_WARN, 0.24)

                near_fall_now = (
                    (pitch_f > T_FALL_EFF) or           # irse hacia adelante
                    (abs(roll_f) > 0.32) or
                    ((walk_timer > SAFE_GRACE_S) and
                     (pitch_f > T_WARN_EFF) and
                     (gait_gain < 0.15))
                )

                # Sólo usamos gait_gain bajo como motivo de RECOVER
                # cuando el torso se va hacia ADELANTE (pitch_f > 0.0).
                if near_fall_now or (gait_gain <= 0.10 and pitch_f > 0.0) or pose["on"]:
                    state = "RECOVER"
                    recover_timer = 0.0
                    bal_prev.clear()
                    LOG.info("gait", "[RECOVER] near_fall/grace/gain<=0/pose_on")
                    for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r",
                              "j05_ankle_roll_l", "j11_ankle_roll_r"):
                        bal_prev[j] = 0.0

                elif not gait_enable:
                    state = "STAND"



        elif state == "PRELEAN":
            prelean_timer += dt
            if not gait_enable or pose["on"]:
                state = "STAND"
            else:
                PITCH_MAX      = 0.25
                PITCH_MAX_NEG  = 0.28

                # si ya se fue MUCHO hacia atrás -> RECOVER
                if pitch_f <= -PITCH_MAX_NEG:
                    state = "RECOVER"
                    recover_timer = 0.0
                    bal_prev.clear()
                    for j in ("j04_ankle_pitch_l", "j10_ankle_pitch_r",
                              "j05_ankle_roll_l", "j11_ankle_roll_r"):
                        bal_prev[j] = 0.0

                # si ya llevamos el tiempo de prelean y no estamos locos de roll -> WALK
                elif (prelean_timer >= PRELEAN_TIME) and (abs(roll_f) < R_STAB):
                    state = "WALK"
                    walk_timer = 0.0
                    LOG.info("gait", f"Transición PRELEAN -> WALK (pitch_f={pitch_f:+.3f})")

            # --- empuje de cadera/tobillo de PRELEAN (torso hacia ADELANTE) ---
            s = min(1.0, prelean_timer / PRELEAN_TIME)
            fade = (max(0.0, 1.0 - max(0.0, (pitch_f - 0.08)) / 0.10)) if (pitch_f >= 0.0) else 1.0
            s *= fade

            # IMPORTANTE: en este robot, hip_pitch NEGATIVO = torso hacia adelante.
            if pitch_f < -0.05:
                # Va inclinándose (según IMU), usamos más cadera pero casi nada de tobillo
                hip_push = -PRELEAN_HIP * 2.0
                ank_bias = +0.005   # antes +0.030
            elif pitch_f > +0.05:
                # Ya algo adelantado: empuje suave
                hip_push = -PRELEAN_HIP * 0.5
                ank_bias = +0.000
            else:
                # Cerca de neutro
                hip_push = -PRELEAN_HIP
                ank_bias = +0.002

            for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
                if hip in target:
                    target[hip] += hip_push * s
            for ank in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
                if ank in target:
                    target[ank] += ank_bias



        
        elif state == "RECOVER":
            # Apagamos gait mientras recuperamos
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

            # --- Lógica de salida de RECOVER ---
            MIN_RECOVER_S = 0.35      # tiempo mínimo en RECOVER
            GY_OK = 0.25              # rad/s aprox (≈ 14°/s)

            gy_val = gy if gyro else 0.0

            # Torso casi vertical, sin irse hacia atrás, poco roll y poca velocidad
            good_pitch = (abs(pitch_f) < 0.06 and pitch_f > -0.01)
            good_roll  = (abs(roll_f)  < 0.08)
            good_gyro  = (abs(gy_val)  < GY_OK)

            if (recover_timer > MIN_RECOVER_S
                    and good_pitch
                    and good_roll
                    and good_gyro):
                recover_hold_ok_t += dt
            else:
                recover_hold_ok_t = 0.0

            # Necesitamos un ratito estable antes de soltar
            if recover_hold_ok_t >= 0.18:
                state = "STAND"
                stable_t = 0.0
                bal_prev.clear()
                recover_hold_ok_t = 0.0

            # Si por cualquier razón caímos en RECOVER con un pitch_f negativo pequeño,
            # lo cancelamos y seguimos caminando o de pie.
            #if state == "RECOVER" and pitch_f < 0.0 and abs(pitch_f) < 0.18:
            #    LOG.info("gait", f"[PATCH] cancel RECOVER (pitch_f={pitch_f:+.3f})")
            #    if gait_enable:
            #        state = "WALK"
            #    else:
            #        state = "STAND"
            #    recover_timer = 0.0
#
        # 5) Salidas por estado
        if state == "STAND":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

        elif state == "PRELEAN":
            # Aún no arrancamos el gait cíclico
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

            # Progreso lineal de la inclinación
            s = min(1.0, prelean_timer / PRELEAN_TIME)

            # Cadera: un poco hacia adelante para cargar el COM
            PRELEAN_HIP = 0.030  # ~1.7°
            for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
                if hip in target:
                    target[hip] += PRELEAN_HIP * s

            # Tobillos: leve dorsiflexión (puntitas arriba) fija
            for ank in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
                if ank in target:
                    target[ank] += +0.010

        elif state == "WALK":
            desired_on = True

            # Sesgo base: torso un poco hacia ADELANTE
            HIP_BASE = 0.0  # un pelín, menos agresivo que antes

            # Corrección extra si el torso se nos va hacia "atrás" según la IMU
            back = max(0.0, -pitch_f)  # sólo cuenta cuando pitch_f < 0
            extra_hip = 0.0
            if back > 0.02:
                # Normaliza cuánto se está yendo (satura alrededor de ~0.20 rad)
                back_norm = min(1.0, (back - 0.02) / (0.20 - 0.02))
                # NEGATIVO = torso hacia ADELANTE en este robot
                extra_hip = -(0.015 + 0.045 * back_norm)   # ~[-0.015 .. -0.060]

            for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
                if hip in target:
                    target[hip] += HIP_BASE + extra_hip

            extra_ank = 0.0

            LOG.info(
                "gait",
                f"[PITCH_CORR] pitch_f={pitch_f:+.3f} HIP_BASE={HIP_BASE:+.3f} "
                f"extra_hip={extra_hip:+.3f} extra_ank={extra_ank:+.3f}"
            )

            state_before = state
            recover_before = recover_timer

            (walk_timer,
             gait_soft_t,
             STRIDE_BASE,
             steps_taken,
             gait_gain,
             stride_gain,
             step_ctx,
             _tele_t,
             state_next,
             recover_next,
             left_is_swing,
             ) = update_walk_outputs(
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
             )

            # Parche: el estado global WALK/RECOVER lo controla el FSM principal,
            # NO el walker. Ignoramos cualquier RECOVER que venga de update_walk_outputs.
            if state_before == "WALK" and state_next == "RECOVER":
                LOG.info(
                    "gait",
                    f"[PATCH_WALK_REC] ignorando RECOVER de update_walk_outputs "
                    f"(pitch_f={pitch_f:+.3f}, roll_f={roll_f:+.3f})"
                )
                state = state_before
                recover_timer = recover_before
            else:
                state = state_next
                recover_timer = recover_next





        elif state == "RECOVER":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

            # gy_val = componente pitch del gyro (ya la traes como gy)
            gy_val = gy if gyro else 0.0

            recover_timer = recover_step(
                dt,
                pitch_f,
                roll_f,
                gy_val,
                bool(gyro),
                target,
                base_pose,
                _rec_pose,
                recover_timer,
                LOG,
            )
        # Aplica on/off sólo si cambia
        if desired_on != walker_last_on:
            walker.toggle(desired_on)
            walker_last_on = desired_on
            LOG.info("gait", f"Walker: {'ON' if desired_on else 'OFF'} | state={state}")
        if state != "WALK":
            walker.set_global_gain(0.0)
            walker.toggle(False)

        # 6) Marcha (walker)
        gait_off = walker.step(dt, {"pitch": pitch, "roll": roll}, {}) or {}
        if gait_off and (step_count % 3 == 0):
            LOG.info(
                "gait",
                f"phase={getattr(walker,'phase',0.0):.2f} "
                f"Δ={len(gait_off)} joints g={getattr(walker,'_global_gain',1.0):.2f}",
            )

        if gait_off:
            apply_gait_offsets(gait_off,
                target,
                base_pose,
                stride_gain,
                pitch_f,
                gyro,
                gy,
                walker,
                left_is_swing,)

        # ===================== POSE: levantar pierna con transferencia =====================
        update_pose(dt, pose, target, base_pose, LOG)

        # 7) Límites de articulaciones + clamps (delegado a joint_limits.py)
        target, state, recover_timer = apply_joint_limits_and_clamps(
            target=target,
            pose=pose,
            limits=limits,
            clamp_ctx=clamp_ctx,
            state=state,
            CLAMP_TAU=CLAMP_TAU,
            dt=dt,
            LOG=LOG,
            recover_timer=recover_timer,)

        # 8) Aplicar
        for n, m in motors.items():
            m.setPosition(target[n])

        # === GPS / dirección ===
        if not hasattr(walker, "dir"):
            walker.dir = +1.0  # por defecto

        if gps:
            x, y, z = gps.getValues()
            if _x_prev is not None:
                dx = x - _x_prev
                _dx_acc = 0.9 * _dx_acc + 0.1 * dx
                _dx_avg = _dx_acc / max(1e-6, dt)
            _x_prev = x

        # 9) Log periódico (delegado a telemetry.py)
        last_log_t = periodic_log(t,
            last_log_t,
            log_cfg,
            LOG,
            state,
            walker,
            pitch,
            roll,
            step_count,
            sample_joints,
            target,
            gps,)

if __name__ == "__main__":
    run()
