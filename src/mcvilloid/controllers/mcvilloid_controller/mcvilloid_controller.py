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

    # --- Teclado (W/S para on/off del gait) ---
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

    gyro = robot.getDevice("gyro")
    if gyro: gyro.enable(TIME_STEP)

    gps = robot.getDevice('gps')
    gps.enable(TIME_STEP)

    # --- Motores (desde params.json) ---
    motor_names = params.get("motors", [])
    motors = {}
    for n in motor_names:
        dev = robot.getDevice(n)
        if dev: motors[n] = dev
        else:   LOG.warn("boot", f"Motor '{n}' NOT FOUND")
    LOG.info("boot", f"Motores OK: {sorted(motors.keys())}")

    # Límites de velocidad por tipo
    ank_vel = float(limits.get("ankle_vel", 1.0))
    hip_vel = float(limits.get("hip_vel",   1.8))
    for name, m in motors.items():
        if "ankle_" in name:
            m.setVelocity(ank_vel)
        elif any(s in name for s in ["hip_", "knee_", "yaw_"]):
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
    if params.get("gait", {}).get("enabled_on_start", True):
        walker.toggle(False)
    LOG.info("gait", f"Walker: {'ON' if getattr(walker,'on', False) else 'OFF'}")

    # Estado
    step_count    = 0
    t             = 0.0
    last_log_t    = 0.0
    sample_joints = ["j03_knee_pitch_l", "j09_knee_pitch_r", "j04_ankle_pitch_l"]

    # ======================== FSM ========================
    state = "STAND"
    gait_enable = False
    walker_last_on = False

    clamp_hit = False
    clamp_timer = 0.0

    prelean_timer = 0.0
    recover_timer = 0.0

    PRELEAN_TIME = 0.0  # s (desactivado)
    PRELEAN_HIP  = 0.0  # rad

    # Umbrales
    T_ENTER = 0.08
    T_EXIT  = 0.06
    T_FALL  = 0.25
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
            if "ankle_" in j:   # ← SOLO tobillos cuentan para RECOVER por clamp
                ankle_clamp_hit = True
        return clamped

    # ---- Balance: límites y rate-limit (pitch vs roll) ----
    BAL_PITCH_LIM = 0.12   # rad máx ankle_pitch por balance
    BAL_ROLL_LIM  = 0.10   # rad máx ankle_roll  por balance
    BAL_RATE_MAX_PITCH = 0.50  # rad/s (un poco más suave)
    BAL_RATE_MAX_ROLL  = 0.35  # rad/s
    bal_prev = {}                # {joint: último offset aplicado}
    def bal_slew(prev, tgt, vmax, dt):
        up = prev + vmax*dt
        dn = prev - vmax*dt
        return up if tgt > up else dn if tgt < dn else tgt


    # ---------------------- Bucle principal ----------------------
    while robot.step(TIME_STEP) != -1:
        step_count += 1
        t += dt
        target = base_pose.copy()
        gait_gain = 1.0  # default; se recalcula con guards más abajo
        ankle_clamp_hit = False
        ankle_clamp_timer = 0.0

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
            key = kb.getKey()

        # 1) IMU
        pitch = 0.0; roll = 0.0
        if imu:
            rpy = imu.getRollPitchYaw()
            roll, pitch = rpy[0], rpy[1]
            LOG.debug("imu", f"r={roll:+.3f} p={pitch:+.3f}")
        if gyro:
            gx, gy, gz = gyro.getValues()  # rad/s
            _pitch_rate = gy

        # 2) Balance (del controlador de balance)
        bal_off = bal.step(dt, pitch, roll)
        if bal_off and (("j04_ankle_pitch_l" in bal_off) or ("j10_ankle_pitch_r" in bal_off)):
            ap_l = bal_off.get("j04_ankle_pitch_l", 0.0)
            ap_r = bal_off.get("j10_ankle_pitch_r", 0.0)
            ar_l = bal_off.get("j05_ankle_roll_l",  0.0)
            ar_r = bal_off.get("j11_ankle_roll_r",  0.0)
            LOG.info("balance", f"Δankle L(p:{ap_l:+.3f}, r:{ar_l:+.3f}) R(p:{ap_r:+.3f}, r:{ar_r:+.3f})")

        # 2.1) Post-proceso del balance: SIN inversión de ankle_pitch + límites + rate-limit
        bal_post = {}
        for j, off in (bal_off or {}).items():
            if "ankle_pitch" in j:
                off = max(-BAL_PITCH_LIM, min(BAL_PITCH_LIM, off))
                vmax = BAL_RATE_MAX_PITCH
            elif "ankle_roll" in j:
                off = max(-BAL_ROLL_LIM, min(BAL_ROLL_LIM, off))
                vmax = BAL_RATE_MAX_ROLL
            else:
                vmax = 0.35
            prev = bal_prev.get(j, 0.0)
            off  = bal_slew(prev, off, vmax, dt)
            bal_prev[j] = off
            if j in target:
                bal_post[j] = off
        for j, off in bal_post.items():
            target[j] += off

        # 3) Guards de amplitud (gait_gain) por IMU
        p = abs(pitch); r = abs(roll)
        if p <= 0.08: g_pitch = 1.0
        elif p >= 0.15: g_pitch = 0.0
        else:
            t_norm = (p - 0.08) / (0.15 - 0.08)
            g_pitch = (1.0 - t_norm)**2
        if r <= 0.06: g_roll = 1.0
        elif r >= 0.18: g_roll = 0.0
        else:
            t_norm = (r - 0.06) / (0.18 - 0.06)
            g_roll = (1.0 - t_norm)**2
        gait_gain = min(g_pitch, g_roll)  # sin cap artificial

        # 4) FSM
        T_WARN = 0.12
        can_walk     = abs(pitch) < T_ENTER
        should_stand = abs(pitch) < T_EXIT
        near_fall = (abs(pitch) > T_FALL) or (abs(roll) > 0.25) or (abs(pitch) > T_WARN and gait_gain < 0.4)

        if state == "STAND":
            if near_fall:
                state = "RECOVER"; recover_timer = 0.0; bal_prev.clear()
            elif gait_enable and can_walk:
                state = "PRELEAN"; prelean_timer = 0.0

        elif state == "WALK":
            if near_fall or gait_gain <= 0.0:
                state = "RECOVER"; recover_timer = 0.0; bal_prev.clear()
            elif not gait_enable:
                state = "STAND"

        elif state == "PRELEAN":
            prelean_timer += dt
            if not gait_enable:
                state = "STAND"
            elif prelean_timer >= PRELEAN_TIME:
                state = "WALK"

        elif state == "RECOVER":
            recover_timer += dt
            if recover_timer >= RECOVER_TIME and should_stand:
                state = "STAND"

        # 5) Salidas por estado
        if state == "STAND":
            desired_on = False
            walker.set_global_gain(0.0)

        elif state == "PRELEAN":
            desired_on = False
            walker.set_global_gain(0.0)

        elif state == "WALK":
            desired_on = True
            if getattr(walker, "dir", None) is None:
                walker.dir = 1.0
            walker.set_global_gain(gait_gain)
            # sin sesgos adicionales (volvemos a la base estable)

        elif state == "RECOVER":
            desired_on = False
            walker.set_global_gain(0.0)
            bal_prev.clear()
            # Dejar neutral + balance. Sin offsets manuales para no sobrecorregir.

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
            if j in target:
                target[j] += off

        # 7) Clamps finales de referencia
        ank_lim = float(limits.get("ankle_pos", 0.22))
        hip_lim = float(limits.get("hip_pos",   0.18))
        for j, ref in list(target.items()):
            if ("ankle_pitch" in j) or ("ankle_roll" in j):
                target[j] = rclamp_with_flag(j, ref, ank_lim)
            elif ("hip_pitch" in j) or ("hip_roll" in j) or ("knee_pitch" in j):
                target[j] = rclamp_with_flag(j, ref, hip_lim)

        # ---- timers de clamp ----  ←  AQUÍ VA
        if ankle_clamp_hit:
            ankle_clamp_timer += dt
        else:
            ankle_clamp_timer = 0.0

        clamp_hit = False
        ankle_clamp_hit = False

        if state == "WALK" and ankle_clamp_timer >= CLAMP_TAU:
            state = "RECOVER"; recover_timer = 0.0

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
                x, y, z = gps.getValues()
                LOG.info("gps", f"[GPS] x={x:.3f} z={z:.3f}")

if __name__ == "__main__":
    run()
