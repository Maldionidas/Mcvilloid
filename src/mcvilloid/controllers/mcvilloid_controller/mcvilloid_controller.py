# src/mcvilloid/controllers/mcvilloid_controller/mcvilloid_controller.py
import os, sys, json, math, time
from controller import Robot, Keyboard

HERE = os.path.dirname(__file__)

# --- Imports compatibles con ejecución como script en Webots y como paquete ---
if __package__ in (None, ""):
    if HERE not in sys.path:
        sys.path.append(HERE)
    from balance.balance import BalanceController
    from movement.walking import Walker
    from control_utils import radians_clamp
else:
    from .balance.balance import BalanceController
    from .movement.walking import Walker
    from .control_utils import radians_clamp

# --- Logger simple con rate limit por canal ---
class RateLogger:
    LVL = {"debug": 10, "info": 20, "warn": 30}
    def __init__(self, cfg, now_fn):
        self.enabled  = bool(cfg.get("enabled", True))
        self.level    = self.LVL.get(str(cfg.get("level", "info")).lower(), 20)
        self.rate_hz  = float(cfg.get("rate_hz", 10.0))
        self.period   = 1.0 / max(1e-6, self.rate_hz)
        self.channels = {k: bool(v) for k, v in cfg.get("channels", {}).items()}
        self._last = {}
        self._now  = now_fn
    def _ok(self, ch, lvl):
        if not self.enabled: return False
        if not self.channels.get(ch, True): return False
        if lvl < self.level: return False
        t = self._now()
        if t - self._last.get(ch, -1e9) < self.period: return False
        self._last[ch] = t
        return True
    def debug(self, ch, msg):
        if self._ok(ch, self.LVL["debug"]): print(f"[{ch}] {msg}", flush=True)
    def info(self, ch, msg):
        if self._ok(ch, self.LVL["info"]):  print(f"[{ch}] {msg}", flush=True)
    def warn(self, ch, msg):
        if self._ok(ch, self.LVL["warn"]):  print(f"[{ch}] {msg}", flush=True)

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
    if gyro: gyro.enable(TIME_STEP)

    gps = robot.getDevice('gps')
    if gps: gps.enable(TIME_STEP)

    # --- Motores (desde params.json) ---
    motor_names = params.get("motors", [])
    motors = {}
    for n in motor_names:
        dev = robot.getDevice(n)
        if dev: motors[n] = dev
        else:   LOG.warn("boot", f"Motor '{n}' NOT FOUND")
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
    for n, m in motors.items():
        m.setPosition(base_pose[n])

    # --- Controladores ---
    bal    = BalanceController(params, logger=lambda *a: LOG.info("balance", " ".join(map(str,a))))
    walker = Walker(params, limits,    logger=lambda *a: LOG.info("gait",    " ".join(map(str,a))))

    # Arranque del gait
    walker.toggle(bool(params.get("gait", {}).get("enabled_on_start", False)))
    LOG.info("gait", f"Walker: {'ON' if getattr(walker,'on', False) else 'OFF'}")

    # Estado/diagnóstico
    step_count    = 0
    t             = 0.0
    last_log_t    = 0.0
    sample_joints = ["j03_knee_pitch_l", "j09_knee_pitch_r", "j04_ankle_pitch_l"]

    # ======================== FSM Gait ========================
    state = "STAND"
    gait_enable = False
    walker_last_on = False

    clamp_hit = False
    clamp_timer = 0.0

    prelean_timer = 0.0
    recover_timer = 0.0

    PRELEAN_TIME = 0.40
    PRELEAN_HIP  = -0.06  # rampa hacia flexión (torso un poco adelante)
    gait_soft_t = 0.0
    GAIT_SOFT_S = float(params.get("gait", {}).get("soft_start_s", 2.0))

    # --- Escalado de zancada (ajustada) ---
    STRIDE_BASE   = float(params.get("gait", {}).get("stride_base", 0.92))
    STRIDE_MAX    = float(params.get("gait", {}).get("stride_max",  1.20))
    STRIDE_MIN    = 0.85
    STRIDE_WARM_S = 1.0  # s para “soltar” la zancada al arrancar WALK

    # Umbrales IMU (se usarán sobre señales filtradas para decisiones)
    T_ENTER = 0.10   # antes 0.08
    T_EXIT  = 0.08   # antes 0.06
    T_FALL  = 0.34   # antes 0.25
    T_WARN  = 0.18   # nuevo (para logs/decisiones suaves)

    ankle_clamp_hit = False
    ankle_clamp_timer = 0.0
    CLAMP_TAU    = 0.25
    RECOVER_TIME = 0.25

    # --- helper: clamp con log ---
    last_clamp_log = {}
    def rclamp_with_flag(j, x, lim, cooldown_s=0.25):
        nonlocal clamp_hit, ankle_clamp_hit
        lim = abs(lim)
        clamped = max(-lim, min(lim, x))
        if clamped != x:
            now = time.monotonic()
            last = last_clamp_log.get(j, -1e9)
            if now - last >= cooldown_s:
                LOG.info("clamp", f"[CLAMP] {j}: {x:+.3f} -> {clamped:+.3f} (lim ±{lim:.2f})")
                last_clamp_log[j] = now
            clamp_hit = True
            if "ankle_" in j:
                ankle_clamp_hit = True
        return clamped

    # ---- Balance: límites y rate-limit (pitch vs roll) ----
    BAL_PITCH_LIM = 0.09
    BAL_ROLL_LIM  = 0.07
    BAL_RATE_MAX_PITCH = 0.5
    BAL_RATE_MAX_ROLL  = 0.22
    bal_prev = {}
    def bal_slew(prev, tgt, vmax, dt):
        up = prev + vmax*dt
        dn = prev - vmax*dt
        return up if tgt > up else dn if tgt < dn else tgt

    # --- IMU smoothing (para decisiones/guards) ---
    TAU_IMU = 0.12  # s
    pitch_f = 0.0
    roll_f  = 0.0
    imu_f_ready = False

    # --- Histeresis de estabilidad para autorizar WALK (ajustada) ---
    stable_t = 0.0
    STAB_T = 0.28
    P_STAB = 0.045  # rad
    R_STAB = 0.055  # rad

    # --- Cronómetro y warm-up de pasos ---
    walk_timer = 0.0
    prev_phase = 0.0
    steps_taken = 0
    WARM_STEPS = 3  # limitar zancada y ganancia en los 3 primeros pasos

    # ======================== FSM POSE (levantar pierna estática) ========================
    pose = {"on": False, "side": "L", "phase": "IDLE", "t": 0.0}
    # --- Estado para RECOVER (hold y detección de mejora) ---
    recover_hold_ok_t = 0.0   # tiempo acumulado con pitch en zona segura
    pitch_f_prev = 0.0


    # ---------------------- Bucle principal ----------------------
    _rec_hist = []  # historial corto de pitch_f para criterio de salida de RECOVER (tendencia)
    while robot.step(TIME_STEP) != -1:
        step_count += 1
        t += dt
        target = base_pose.copy()
        gait_gain = 1.0
        stride_gain = 1.0
        ankle_clamp_hit = False

        # --- Teclas ---
        key = kb.getKey()
        while key != -1:
            if key in (ord('W'), ord('w')):
                gait_enable = True
                LOG.info("gait","gait_enable = True")
            elif key in (ord('S'), ord('s')):
                gait_enable = False
                LOG.info("gait","gait_enable = False")
            elif key in (ord('A'), ord('a')):  # backward
                walker.dir = -1.0
                LOG.info("gait", "Direction: BACKWARD")
            elif key in (ord('D'), ord('d')):  # forward
                walker.dir = +1.0
                LOG.info("gait", "Direction: FORWARD")

            # --- POSE estática: levantar pierna ---
            elif key == ord('F'):  # levantar pierna IZQ
                pose.update({"on": True, "side": "L", "phase": "SHIFT", "t": 0.0})
                walker.toggle(False)
                walker.set_global_gain(0.0)
                LOG.info("gait", "[POSE] Levantar IZQ: SHIFT")
            elif key == ord('G'):  # levantar pierna DER
                pose.update({"on": True, "side": "R", "phase": "SHIFT", "t": 0.0})
                walker.toggle(False)
                walker.set_global_gain(0.0)
                LOG.info("gait", "[POSE] Levantar DER: SHIFT")
            elif key == ord('H'):  # cancelar pose
                pose.update({"on": False, "phase": "IDLE", "t": 0.0})
                LOG.info("gait", "[POSE] cancelada")
            elif key in (ord('+'), ord('=')):  # aumentar zancada base
                STRIDE_BASE = min(STRIDE_MAX, STRIDE_BASE + 0.05)
                LOG.info("gait", f"[STRIDE] base -> {STRIDE_BASE:.2f}")
            elif key in (ord('-'), ord('_')):  # reducir zancada base
                STRIDE_BASE = max(STRIDE_MIN, STRIDE_BASE - 0.05)
                LOG.info("gait", f"[STRIDE] base -> {STRIDE_BASE:.2f}")

            key = kb.getKey()

        # 1) IMU
        pitch = 0.0; roll = 0.0
        if imu:
            rpy = imu.getRollPitchYaw()
            roll, pitch = rpy[0], rpy[1]
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

        # Acumulador de estabilidad (no castigues en PRELEAN)
        if state != "PRELEAN":
            if abs(pitch_f) < P_STAB and abs(roll_f) < R_STAB:
                stable_t = min(STAB_T, stable_t + dt)
            else:
                stable_t = max(0.0, stable_t - 2.0*dt)
        # en PRELEAN dejamos estable el stable_t (congelado)

        # 2) Balance
        bal_off = bal.step(dt, pitch, roll)
        b_scale = 1.0 if state != "WALK" else max(0.4, 1.0 - getattr(walker, "_global_gain", 0.0) * 0.7)
        bal_post = {}
        for j, off in (bal_off or {}).items():
            if "ankle_pitch" in j:
                off = -off
                off = max(-BAL_PITCH_LIM, min(BAL_PITCH_LIM, off))
                vmax = BAL_RATE_MAX_PITCH
            elif "ankle_roll" in j:
                if state == "WALK" and walk_timer < 0.40:
                    off = 0.0
                off = max(-BAL_ROLL_LIM, min(BAL_ROLL_LIM, off))
                vmax = BAL_RATE_MAX_ROLL
            else:
                vmax = 0.35

            off *= b_scale
            prev = bal_prev.get(j, 0.0)
            off  = bal_slew(prev, off, vmax, dt)
            bal_prev[j] = off
            if j in target:
                bal_post[j] = off

        # --- AJUSTE CRÍTICO: amordazar balance en RECOVER y permitir solo dorsiflexión en tobillo ---
        if state == "RECOVER" and bal_post:
            # reducir ganancia global de balance durante la recuperación
            for j in list(bal_post.keys()):
                bal_post[j] *= 0.30
            # tobillo pitch: solo dorsiflexión (>=0), cap suave
            for ap in ("j04_ankle_pitch_l","j10_ankle_pitch_r"):
                if ap in bal_post:
                    bal_post[ap] = max(0.0, min(0.04, bal_post[ap]))

        # aplicar balance
        for j, off in bal_post.items():
            target[j] += off

        if bal_post and (("j04_ankle_pitch_l" in bal_post) or ("j10_ankle_pitch_r" in bal_post)
                         or ("j05_ankle_roll_l" in bal_post) or ("j11_ankle_roll_r" in bal_post)):
            ap_l = bal_post.get("j04_ankle_pitch_l", 0.0)
            ap_r = bal_post.get("j10_ankle_pitch_r", 0.0)
            ar_l = bal_post.get("j05_ankle_roll_l",  0.0)
            ar_r = bal_post.get("j11_ankle_roll_r",  0.0)
            LOG.info("balance", f"Δankle L(p:{ap_l:+.3f}, r:{ar_l:+.3f}) R(p:{ap_r:+.3f}, r:{ar_r:+.3f})")

        # 3) Guards de amplitud (gait_gain) por IMU
        p_abs = abs(pitch_f); r_abs = abs(roll_f)

        # g_pitch
        if p_abs <= 0.05: g_pitch = 1.0
        elif p_abs >= 0.10: g_pitch = 0.0
        else:
            t_norm = (p_abs - 0.05) / (0.10 - 0.05)
            g_pitch = (1.0 - t_norm)**2
        if pitch_f < 0.0:
            g_pitch *= max(0.0, 1.0 - 6.0 * (-pitch_f))

        # g_roll
        if r_abs <= 0.06: g_roll = 1.0
        elif r_abs >= 0.18: g_roll = 0.0
        else:
            t_norm = (r_abs - 0.06) / (0.18 - 0.06)
            g_roll = (1.0 - t_norm)**2
        g_roll *= max(0.0, 1.0 - 2.8 * max(0.0, r_abs - 0.06))

        gait_gain = min(g_pitch, g_roll)

        # 4) FSM Gait principal
        can_walk     = abs(pitch_f) < T_ENTER
        should_stand = abs(pitch_f) < T_EXIT
        near_fall = (abs(pitch_f) > T_FALL) or (abs(roll_f) > 0.32) \
            or (abs(pitch_f) > T_WARN and gait_gain < 0.28)

        if state == "STAND":
            walk_timer = 0.0
            if near_fall:
                state = "RECOVER"; recover_timer = 0.0; bal_prev.clear()
                # reset memoria de balance en tobillos/roll al entrar a RECOVER
                for j in ("j04_ankle_pitch_l","j10_ankle_pitch_r","j05_ankle_roll_l","j11_ankle_roll_r"):
                    bal_prev[j] = 0.0
            elif gait_enable and (stable_t >= STAB_T) and not pose["on"]:
                state = "PRELEAN"; prelean_timer = 0.0

        elif state == "WALK":
            if near_fall or gait_gain <= 0.0 or pose["on"]:
                state = "RECOVER"; recover_timer = 0.0; bal_prev.clear()
                LOG.info("gait", "[RECOVER] near_fall/gain<=0/pose_on")
                for j in ("j04_ankle_pitch_l","j10_ankle_pitch_r","j05_ankle_roll_l","j11_ankle_roll_r"):
                    bal_prev[j] = 0.0
            elif not gait_enable:
                state = "STAND"

        elif state == "PRELEAN":
            prelean_timer += dt
            if not gait_enable or pose["on"]:
                state = "STAND"
            else:
                stable_ok_prelean = (abs(roll_f) < R_STAB and abs(pitch_f) < 0.09)
                if prelean_timer >= PRELEAN_TIME and stable_ok_prelean:
                    state = "WALK"
                    walk_timer = 0.0
                    LOG.info("gait", "Transición PRELEAN -> WALK")

        elif state == "RECOVER":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

            # parámetros de catch-pose ABSOLUTOS
            REC_HIP_ABS   = -0.08   # torso adelante (negativo en tu convención)
            REC_KNEE_ABS  = +0.12   # flexión suave
            REC_ANK_ABS   = +0.06   # dorsiflexión clara (sube puntas)

            # refuerzo si sigue yéndose hacia atrás
            if pitch_f < -0.20:
                REC_HIP_ABS  = -0.11
                REC_KNEE_ABS = +0.18
                REC_ANK_ABS  = +0.08

            # aplicar ABSOLUTOS (no relativos a base)
            for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                if hip in target: target[hip] = REC_HIP_ABS
            for knee in ("j03_knee_pitch_l","j09_knee_pitch_r"):
                if knee in target: target[knee] = REC_KNEE_ABS
            for ank in ("j04_ankle_pitch_l","j10_ankle_pitch_r"):
                if ank in target:  target[ank]  = max(target[ank], REC_ANK_ABS)

            # limitar fuerte el roll de tobillo durante recuperación
            for j in ("j05_ankle_roll_l","j11_ankle_roll_r"):
                if j in target:
                    target[j] = max(-0.03, min(0.03, target[j]))

            # PD de cadera más enérgico para llevar pitch a +0.02
            pitch_target = +0.02
            Kp, Kd = 0.20, 0.04
            gy = gy if gyro else 0.0
            u = -Kp * (pitch_target - pitch_f) - Kd * gy
            u = max(-0.12, min(0.12, u))
            for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                if hip in target: target[hip] += u  # sobre la pose absoluta

            # lógica de "hold": no sueltes hasta que se recupere de verdad
            # condición: |pitch_f| < 0.06 durante 0.15 s continuos
            if abs(pitch_f) < 0.06:
                recover_hold_ok_t += dt
            else:
                recover_hold_ok_t = 0.0

            # salida de RECOVER solo cuando hubo ventana estable
            if recover_hold_ok_t >= 0.15 and abs(roll_f) < 0.08:
                state = "STAND"
                stable_t = 0.0
                bal_prev.clear()
                recover_hold_ok_t = 0.0

            # guarda pitch filtrado previo (útil si luego quieres lógica adicional)
            pitch_f_prev = pitch_f


        # 5) Salidas por estado
        if state == "STAND":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

        elif state == "PRELEAN":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0
            s = min(1.0, prelean_timer / PRELEAN_TIME)
            for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
                if hip in target:
                    target[hip] += PRELEAN_HIP * s

        elif state == "WALK":
            desired_on = True
            walk_timer += dt

            # soft gain de entrada al gait (0 → 1 en GAIT_SOFT_S)
            gait_soft_t = min(GAIT_SOFT_S, gait_soft_t + dt)
            soft_gain = min(1.0, max(0.0, gait_soft_t / GAIT_SOFT_S))

            phase = getattr(walker, "phase", 0.0)

            # detectar cruce de fase ~1->0 para contar pasos completos
            if prev_phase > 0.90 and phase < 0.10:
                steps_taken += 1
            prev_phase = phase

            # limitador para primeros pasos (evita zancada larga y caída hacia atrás)
            if steps_taken < WARM_STEPS:
                # 1er paso 0.55x, 2º 0.70x, 3º 0.85x
                micro = 0.55 + 0.15 * steps_taken
            else:
                micro = 1.0

            # >>> Cálculo de stride_gain consolidado en WALK <<<
            soft_str   = min(1.0, max(0.0, walk_timer / STRIDE_WARM_S))
            stab_p     = max(0.0, 1.0 - min(0.12, abs(pitch_f)) / 0.12)
            stab_r     = max(0.0, 1.0 - min(0.12, abs(roll_f))  / 0.12)
            stability  = stab_p * stab_r
            stride_gain = STRIDE_BASE + 0.35 * (soft_str * stability)
            stride_gain = max(STRIDE_MIN, min(STRIDE_MAX, stride_gain))
            stride_gain *= micro  # aplicar warm-up

            # freno adicional por inestabilidad (pitch- / roll alto)
            instab_pitch = max(0.0, -pitch)            # solo si va hacia atrás
            instab_roll  = max(0.0, abs(roll_f) - 0.06)
            k_pitch = 1.0 - min(0.6,  instab_pitch / 0.10 * 0.6)
            k_roll  = 1.0 - min(0.6,  instab_roll  / 0.14 * 0.6)
            k_instab = max(0.5, min(1.0, k_pitch * k_roll))
            stride_gain *= k_instab
            if (pitch_f < -0.02) or (gyro and gy < -0.12):
                stride_gain = min(stride_gain, 0.98)

            # leve flexión media en cadera según stride_gain
            HIP_MIDFF = min(0.020, 0.030 * stride_gain)  # máx 0.02 rad
            for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                target[hip] += HIP_MIDFF * 0.5

            pitch_rate = gy if gyro else 0.0

            # refuerzo de COM hacia delante en 8–18% de fase (apoyo inicial)
            if 0.08 < phase < 0.18:
                right_is_support = (phase < 0.5)
                hip_sup = "j06_hip_pitch_r" if right_is_support else "j00_hip_pitch_l"
                ank_sup = "j10_ankle_pitch_r" if right_is_support else "j04_ankle_pitch_l"
                if hip_sup in target: target[hip_sup] += +0.012
                if ank_sup in target: target[ank_sup] += -0.018

            # freno por tendencia hacia atrás (único lugar)
            trend_brake = 0.60 if (pitch < -0.03 or pitch_rate < -0.12) else 1.0
            walker.set_global_gain(
                min(0.68, max(0.28, (gait_gain * soft_gain * trend_brake) * micro))
            )

            # Telemetría debug (cada 0.5 s)
            try:
                _tele_t
            except NameError:
                _tele_t = 0.0
            _tele_t += dt
            if _tele_t >= 0.50:
                _tele_t = 0.0
                LOG.info("gait", (
                    f"[TUNE] phase={getattr(walker,'phase',0.0):.2f} "
                    f"g={getattr(walker,'_global_gain',1.0):.2f} "
                    f"soft={soft_gain:.2f} stride={stride_gain:.2f} "
                    f"brake={trend_brake:.2f} "
                    f"p_f={pitch_f:+.3f} r_f={roll_f:+.3f} "
                    f"strideBase={STRIDE_BASE:.2f}"
                ))

            # 2) Feed-forward estable cuando pitch negativo
            if (pitch < -0.035) or (pitch_rate < -0.30):
                sup = "L" if (0.0 <= phase < 0.5) else "R"
                EXTRA_PF   = -0.024
                EXTRA_HIPF =  0.030
                EXTRA_KNEE =  0.016
                ank_sup = "j04_ankle_pitch_l" if sup=="L" else "j10_ankle_pitch_r"
                hip_sup = "j00_hip_pitch_l"   if sup=="L" else "j06_hip_pitch_r"
                knee_sup= "j03_knee_pitch_l"  if sup=="L" else "j09_knee_pitch_r"
                if ank_sup in target:  target[ank_sup] += EXTRA_PF
                if hip_sup in target:  target[hip_sup] += EXTRA_HIPF
                if knee_sup in target: target[knee_sup]+= EXTRA_KNEE

            # 3) Clamp local del tobillo si pitch muy negativo
            if pitch < -0.08:
                BAL_PITCH_LIM_LOCAL = 0.06
                for j in ("j04_ankle_pitch_l","j10_ankle_pitch_r"):
                    if j in target:
                        ref = target[j]
                        target[j] = max(-BAL_PITCH_LIM_LOCAL, min(BAL_PITCH_LIM_LOCAL, ref))

            # 3b) Clamp local de ankle_roll si hay roll alto
            if abs(roll_f) > 0.08:
                BAL_ROLL_LIM_LOCAL = 0.05
                for j in ("j05_ankle_roll_l","j11_ankle_roll_r"):
                    if j in target:
                        ref = target[j]
                        target[j] = max(-BAL_ROLL_LIM_LOCAL, min(BAL_ROLL_LIM_LOCAL, ref))

            # 4) Bias de cadera proporcional al pitch (solo negativo)
            negp = max(0.0, -pitch)
            HIP_FF_K, HIP_FF_MAX = 0.85, 0.07
            hip_add = min(HIP_FF_MAX, HIP_FF_K * negp)
            for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                if hip in target: target[hip] += hip_add

            # 5) Swing retraction cerca de heel-strike
            if (phase > 0.82) or (phase < 0.06):
                RETRACT = -0.025
                for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                    if hip in target: target[hip] += RETRACT

            # --- Empuje en TOE-OFF (pie de soporte) ---
            toe = 0.22; W = 0.06
            if abs((phase - toe + 1.0) % 1.0) < W and pitch > -0.02:  # evita empuje si ya vas hacia atrás
                left_is_swing = (phase < 0.5)
                ank_sup = "j10_ankle_pitch_r" if left_is_swing else "j04_ankle_pitch_l"
                hip_sup = "j06_hip_pitch_r"   if left_is_swing else "j00_hip_pitch_l"
                if ank_sup in target: target[ank_sup] += -0.028  # un pelín menos agresivo
                if hip_sup in target: target[hip_sup] += +0.016

            # --- Amortiguador en HEEL-STRIKE ---
            if (phase > 0.96) or (phase < 0.08):
                left_is_striking = (phase < 0.08)
                knee_strike = "j03_knee_pitch_l" if left_is_striking else "j09_knee_pitch_r"
                ank_strike  = "j04_ankle_pitch_l" if left_is_striking else "j10_ankle_pitch_r"
                if knee_strike in target: target[knee_strike] += +0.040
                if ank_strike  in target: target[ank_strike]  += +0.020
                ank_roll_strike = "j05_ankle_roll_l" if left_is_striking else "j11_ankle_roll_r"
                if ank_roll_strike in target:
                    base = base_pose.get(ank_roll_strike, 0.0)
                    target[ank_roll_strike] = 0.6*target[ank_roll_strike] + 0.4*base

            # --- Sesgo lateral en apoyo para “atrapar” el COM ---
            if (phase > 0.45) and (phase < 0.65):
                left_is_support = (phase < 0.5)
                hip_roll_sup = "j07_hip_roll_r" if left_is_support else "j01_hip_roll_l"
                if hip_roll_sup in target:
                    target[hip_roll_sup] += 0.010

        elif state == "RECOVER":
            desired_on = False
            walker.set_global_gain(0.0)
            gait_soft_t = 0.0

            # Acumular tiempo de recuperación
            recover_timer += dt

            # --- Catch pose con rampa de salida ---
            RECOVER_TIME = 0.35
            if recover_timer <= dt:
                _rec_pose = {
                    "j04_ankle_pitch_l": +0.10,
                    "j10_ankle_pitch_r": +0.10,
                    "j00_hip_pitch_l":   +0.08,
                    "j06_hip_pitch_r":   +0.08,
                    "j03_knee_pitch_l":  +0.084,
                    "j09_knee_pitch_r":  +0.084,
                }

            s = max(0.0, 1.0 - (recover_timer / RECOVER_TIME))
            for j, add in _rec_pose.items():
                if j in target:
                    target[j] = base_pose.get(j, 0.0) + add * s

            # Dorsiflexión ABSOLUTA (no relativa a base) para empujar COM adelante
            REC_ANK_ABS = +0.05  # ≈ 2.8°
            for j in ("j04_ankle_pitch_l","j10_ankle_pitch_r"):
                if j in target:
                    # fuerza al menos +0.05 rad (evita que base -0.04 la anule)
                    target[j] = max(target[j], REC_ANK_ABS)

            # --- Pequeño PD en cadera para empujar pitch a ~+0.01 ---
            pitch_target = +0.02
            Kp, Kd = 0.20, 0.04
            gy = gy if gyro else 0.0
            u = -Kp * (pitch_target - pitch_f) - Kd * gy
            u = max(-0.12, min(0.12, u))
            for hip in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                if hip in target:
                    target[hip] += u

            # --- Limitar fuerte el roll de tobillo durante recuperación ---
            for j in ("j05_ankle_roll_l","j11_ankle_roll_r"):
                if j in target:
                    target[j] = max(-0.03, min(0.03, target[j]))

        # Aplica on/off sólo si cambia
        if desired_on != walker_last_on:
            walker.toggle(desired_on)
            walker_last_on = desired_on
            LOG.info("gait", f"Walker: {'ON' if desired_on else 'OFF'} | state={state}")

        # 6) Marcha (walker)
        gait_off = walker.step(dt, {"pitch": pitch, "roll": roll}, {})
        if gait_off and (step_count % 3 == 0):
            LOG.info("gait", f"phase={getattr(walker,'phase',0.0):.2f} Δ={len(gait_off)} joints g={getattr(walker,'_global_gain',1.0):.2f}")
        for j, off in gait_off.items():
            if j not in target:
                continue
            # Más recorrido en cadera; moderado en rodilla; suave en tobillo
            if "hip_pitch" in j:
                scale = stride_gain
            elif "knee_pitch" in j:
                scale = 0.78 + 0.35 * (stride_gain - 1.0)
            elif "ankle_pitch" in j:
                scale = 0.80 + 0.15 * (stride_gain - 1.0)
            else:
                scale = 1.0
            target[j] += off * scale

        # ===================== POSE: levantar pierna con transferencia =====================
        if pose["on"]:
            pose["t"] += dt
            left  = (pose["side"] == "L")
            sup   = "R" if left else "L"
            swing = "L" if left else "R"

            # nombres útiles
            hip_sup   = f"j06_hip_pitch_r" if sup=="R" else f"j00_hip_pitch_l"
            hip_sw    = f"j00_hip_pitch_l" if swing=="L" else f"j06_hip_pitch_r"
            knee_sw   = f"j03_knee_pitch_l" if swing=="L" else f"j09_knee_pitch_r"
            ankP_sup  = f"j10_ankle_pitch_r" if sup=="R" else f"j04_ankle_pitch_l"
            ankR_sup  = f"j11_ankle_roll_r"  if sup=="R" else f"j05_ankle_roll_l"
            ankP_sw   = f"j04_ankle_pitch_l" if swing=="L" else f"j10_ankle_pitch_r"
            ankR_sw   = f"j05_ankle_roll_l"  if swing=="L" else f"j11_ankle_roll_r"

            # Parámetros moderados
            T_SHIFT   = 0.35   # s
            T_UNLOAD  = 0.20   # s
            T_LIFT    = 0.35   # s
            HIP_FLEX  = -1.60  # rad (cadera swing hacia delante)
            KNEE_FLEX = +1.0   # rad
            ANK_DORSI = +0.12  # rad pie swing
            TORSO_FWD = -0.05  # rad (empuja COM adelante)
            ROLL_BIAS = +0.12 if sup=="R" else -0.12
            ANK_SUP_P = -0.04  # plantar leve soporte
            ANK_SUP_R = ROLL_BIAS * 0.6

            phase = pose["phase"]

            if phase == "SHIFT":
                s = min(1.0, pose["t"]/T_SHIFT)
                for hp in ("j00_hip_pitch_l","j06_hip_pitch_r"):
                    if hp in target: target[hp] += TORSO_FWD * s
                if ankR_sup in target: target[ankR_sup] += ANK_SUP_R * s
                if ankR_sw  in target: target[ankR_sw]  -= ANK_SUP_R * s
                if ankP_sup in target: target[ankP_sup] += ANK_SUP_P * s

                if pose["t"] >= T_SHIFT:
                    pose.update({"phase":"UNLOAD","t":0.0})
                    LOG.info("gait","[POSE] UNLOAD")

            elif phase == "UNLOAD":
                s = min(1.0, pose["t"]/T_UNLOAD)
                if ankP_sw in target: target[ankP_sw] = base_pose.get(ankP_sw,0.0) + ANK_DORSI*0.2*s
                if ankP_sup in target: target[ankP_sup] = base_pose.get(ankP_sup,0.0) + ANK_SUP_P
                if pose["t"] >= T_UNLOAD:
                    pose.update({"phase":"LIFT","t":0.0})
                    LOG.info("gait","[POSE] LIFT")

            elif phase == "LIFT":
                s = min(1.0, pose["t"]/T_LIFT)
                if ankP_sup in target: target[ankP_sup] = base_pose.get(ankP_sup,0.0) + ANK_SUP_P
                if ankR_sup in target: target[ankR_sup] = base_pose.get(ankR_sup,0.0) + ANK_SUP_R
                if ankR_sw  in target: target[ankR_sw]  = base_pose.get(ankR_sw,0.0)  - ANK_SUP_R

                if hip_sw in target:  target[hip_sw]  = base_pose.get(hip_sw,0.0)  + HIP_FLEX  * s
                if knee_sw in target: target[knee_sw] = base_pose.get(knee_sw,0.0) + KNEE_FLEX * s
                if ankP_sw in target: target[ankP_sw] = base_pose.get(ankP_sw,0.0) + ANK_DORSI * s

                if pose["t"] >= T_LIFT:
                    pose.update({"phase":"HOLD","t":0.0})
                    LOG.info("gait","[POSE] HOLD")

            elif phase == "HOLD":
                if ankP_sup in target: target[ankP_sup] = base_pose.get(ankP_sup,0.0) + ANK_SUP_P
                if ankR_sup in target: target[ankR_sup] = base_pose.get(ankR_sup,0.0) + ANK_SUP_R
                if ankR_sw  in target: target[ankR_sw]  = base_pose.get(ankR_sw,0.0)  - ANK_SUP_R
                if hip_sw  in target: target[hip_sw]  = base_pose.get(hip_sw,0.0)  + HIP_FLEX
                if knee_sw in target: target[knee_sw] = base_pose.get(knee_sw,0.0) + KNEE_FLEX
                if ankP_sw in target: target[ankP_sw] = base_pose.get(ankP_sw,0.0) + ANK_DORSI
        # =================== FIN POSE ======================================================

        # 7) Clamps finales de referencia
        ank_lim  = float(limits.get("ankle_pos", 0.22))
        hip_lim  = float(limits.get("hip_pos",   0.18))
        knee_lim = float(limits.get("knee_pos",  0.30))

        # Clamps más permisivos durante la pose
        if pose["on"]:
            hip_lim  = max(hip_lim,  1.20)
            knee_lim = max(knee_lim, 1.60)
            ank_lim  = max(ank_lim,  0.22)

        for j, ref in list(target.items()):
            if ("ankle_pitch" in j) or ("ankle_roll" in j):
                target[j] = rclamp_with_flag(j, ref, ank_lim)
            elif ("knee_pitch" in j):
                target[j] = rclamp_with_flag(j, ref, knee_lim)
            elif ("hip_pitch" in j) or ("hip_roll" in j):  # <- fix typo
                target[j] = rclamp_with_flag(j, ref, hip_lim)

        # ---- timers de clamp (acumulador con decadencia) ----
        DECAY = 0.20  # s^-1
        if ankle_clamp_hit:
            ankle_clamp_timer += dt
        else:
            ankle_clamp_timer = max(0.0, ankle_clamp_timer - DECAY*dt)
        ankle_clamp_hit = False

        if state == "WALK" and ankle_clamp_timer >= CLAMP_TAU:
            state = "RECOVER"; recover_timer = 0.0
            LOG.info("gait", "[RECOVER] clamp sostenido en tobillo")

        # 8) Aplicar
        for n, m in motors.items():
            m.setPosition(target[n])

        # 9) Log periódico
        if log_cfg.get("enabled", True) and log_cfg.get("rate_hz", 10) > 0:
            period = 1.0 / float(log_cfg.get("rate_hz", 10))
            if (t - last_log_t) >= period:
                LOG.info("gait", f"state={state} gain={getattr(walker,'_global_gain',1.0):.2f} | pitch={pitch:+.3f}")
                sj = ", ".join(f"{j}={target.get(j,0.0):+.3f}" for j in sample_joints if j in target)
                print(f"[IMU] t={t:6.2f}s pitch={pitch:+.3f} roll={roll:+.3f} steps={step_count} | {sj}", flush=True)
                last_log_t = t
                if gps:
                    x, y, z = gps.getValues()
                    LOG.info("gps", f"[GPS] x={x:.3f} z={z:.3f}")

if __name__ == "__main__":
    run()
