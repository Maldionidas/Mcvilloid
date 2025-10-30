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
        if not self.enabled:
            return False
        if not self.channels.get(ch, True):
            return False
        if lvl < self.level:
            return False
        t = self._now()
        if t - self._last.get(ch, -1e9) < self.period:
            return False
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
    if gyro:
        gyro.enable(TIME_STEP)


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

    # Límites de velocidad
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
    step_count   = 0
    t            = 0.0
    last_log_t   = 0.0
    sample_joints = ["j03_knee_pitch_l", "j09_knee_pitch_r", "j04_ankle_pitch_l"]

        # --- helper: clamp con log (sin depender de 'step') ---
    last_clamp_log = {}  # por articulación

    def rclamp_with_flag(j, x, lim, cooldown_s=0.25):
        lim = abs(lim)
        clamped = max(-lim, min(lim, x))
        if clamped != x:
            now = time.monotonic()
            last = last_clamp_log.get(j, -1e9)
            if now - last >= cooldown_s:
                LOG.info("clamp",f"[CLAMP] {j}: {x:+.3f} -> {clamped:+.3f} (lim ±{lim:.2f})")
                last_clamp_log[j] = now
        return clamped


    pitch_I = 0.0
    Ki_ANK = 0.08     # empieza chico; sube a 0.10 si ves error sostenido
    I_LIM  = 0.25     # evita windup

    # ---------------------- Bucle principal ----------------------
    while robot.step(TIME_STEP) != -1:
        step_count += 1
        t += dt
        target = base_pose.copy()

        # --- Teclas ---
        key = kb.getKey()
        while key != -1:
            if key in (ord('W'), ord('w')):
                walker.toggle(True)
                LOG.info("gait", "Walker: ON")
            elif key in (ord('S'), ord('s')):
                walker.toggle(False)
                LOG.info("gait", "Walker: OFF")
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
        pitch_rate = 0.0
        if gyro:
            gx, gy, gz = gyro.getValues()  # rad/s
            # Suponemos que 'gy' ≈ velocidad de pitch (depende de ejes del modelo)
            pitch_rate = gy


        # 2) Balance
        bal_off = bal.step(dt, pitch, roll)
        # Log de tobillos del balance (compacto)
        if bal_off and (("j04_ankle_pitch_l" in bal_off) or ("j10_ankle_pitch_r" in bal_off)):
            ap_l = bal_off.get("j04_ankle_pitch_l", 0.0)
            ap_r = bal_off.get("j10_ankle_pitch_r", 0.0)
            ar_l = bal_off.get("j05_ankle_roll_l",  0.0)
            ar_r = bal_off.get("j11_ankle_roll_r",  0.0)
            LOG.info("balance", f"Δankle L(p:{ap_l:+.3f}, r:{ar_l:+.3f}) R(p:{ap_r:+.3f}, r:{ar_r:+.3f})")

        for j, off in bal_off.items():
            if j in target:
                target[j] += off

#        # 2.5) Tilt-guard: atenúa zancada y corrige con tobillos
#
#        tilt = pitch  # +: torso adelante, -: torso atrás
#
#        TZ = 0.08
#        TL = 0.30
#        if abs(tilt) <= TZ:
#            gait_gain = 1.0
#        else:
#            gait_gain = max(0.0, 1.0 - (abs(tilt) - TZ) / (TL - TZ))  # ← piso en 0.30
#
#        walker.set_global_gain(gait_gain * gait_gain)
#
#        # PD en tobillo y cadera. OJO con signos: sumamos u_* al target.
#        Kp_ANK = 0.35
#        Kd_ANK = 0.08
#        Ki_ANK = 0.05
#
#        Kp_HIP = 1.0
#        Kd_HIP = 0.12
#
#        # Integrador anti-sesgo (clamp para evitar windup)
#        pitch_I += tilt * dt
#        pitch_I = max(-I_LIM, min(I_LIM, pitch_I))
#
#
#        # Si gy no es pitch rate, cambia por el eje correcto del gyro (ver abajo).
#        u_ank = -(Kp_ANK * tilt + Kd_ANK * (-pitch_rate)) + Ki_ANK * pitch_I
#        u_hip = -(Kp_HIP * tilt + Kd_HIP * (-pitch_rate))
#
#        for jname in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
#            if jname in target:
#                target[jname] += u_ank    # antes: target[jname] -= u_ank
#
#        for jname in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
#            if jname in target:
#                target[jname] += u_hip    # antes: target[jname] -= u_hip


        


        # 3) Marcha (walker)
        gait_off = walker.step(dt, {"pitch": pitch, "roll": roll}, {})
        if gait_off and (step_count % 3 == 0):
            LOG.info("gait", f"phase={getattr(walker,'phase',0.0):.2f} Δ={len(gait_off)} joints g={getattr(walker,'_global_gain',1.0):.2f}")
        for j, off in gait_off.items():
            if j in target:
                target[j] += off

        # 4) Clamps
        ank_lim = float(limits.get("ankle_pos", 0.22))
        hip_lim = float(limits.get("hip_pos",   0.18))
        for j, ref in list(target.items()):
            if ("ankle_pitch" in j) or ("ankle_roll" in j):
                target[j] = rclamp_with_flag(j, ref, ank_lim)
            elif ("hip_pitch" in j) or ("hip_roll" in j) or ("knee_pitch" in j):
                target[j] = rclamp_with_flag(j, ref, hip_lim)

        # 5) Aplicar
        for n, m in motors.items():
            m.setPosition(target[n])

        # 6) Log periódico IMU + algunas juntas
        if log_cfg.get("enabled", True) and log_cfg.get("rate_hz", 10) > 0:
            period = 1.0 / float(log_cfg.get("rate_hz", 10))
            if (t - last_log_t) >= period:
                sj = ", ".join(f"{j}={target.get(j,0.0):+.3f}" for j in sample_joints if j in target)
                print(f"[IMU] t={t:6.2f}s pitch={pitch:+.3f} roll={roll:+.3f} steps={step_count} | {sj}", flush=True)
                last_log_t = t

if __name__ == "__main__":
    run()
