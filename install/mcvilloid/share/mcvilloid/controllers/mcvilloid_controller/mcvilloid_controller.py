"""
Bípedo - estabilización por juntas (v2, FIXED)
- Usa IMU (roll/pitch) + Gyro (wx/wy) → PD real.
- Calibra offset inicial (promedio 1s).
- Arranque suave y rate limiting.
- Corrige NameError (definir ts ANTES de usarlo) y deprecations (getDevice).

Requisitos:
- Tu PROTO debe tener: InertialUnit { name "imu_sensor" } y Gyro { name "gyro" }
- En el .wbt, Pm01 { controller "mcvilloid_controller" customData "..." } (UNA línea)
"""

from controller import Robot
import json, math

# -------------------- Helpers --------------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

DEBUG = True  # logs extra al arrancar

# -------------------- Inicialización --------------------
print("[BOOT] mcvilloid_controller arrancó")

robot = Robot()

# 1) Paso de simulación primero (¡para poder usar 'ts' después!)
ts = int(robot.getBasicTimeStep()) or 16  # ms

# 2) Lee configuración desde customData
try:
    cfg = json.loads(robot.getCustomData() or "{}")
except Exception:
    print("[WARN] customData inválido; usando defaults.")
    cfg = {}
    
    
targets_deg = cfg.get("targets_deg", {"roll": 0.0, "pitch": 0.0})
t_r = math.radians(float(targets_deg.get("roll", 0.0)))
t_p = math.radians(float(targets_deg.get("pitch", 0.0)))

imu_name  = cfg.get("imu_name", "imu_sensor")
gyro_name = cfg.get("gyro_name", "gyro")
rate_hz   = int(cfg.get("rate_hz", 100))

g_roll  = cfg.get("gains", {}).get("roll",  {"kp":6.0, "kd":0.12})
g_pitch = cfg.get("gains", {}).get("pitch", {"kp":7.0, "kd":0.14})

deadband = math.radians(float(cfg.get("deadband_deg", 0.5)))
soft_start_s = float(cfg.get("soft_start_s", 1.5))

signs = cfg.get("signs", {"invert_roll": False, "invert_pitch": False})
invert_roll  = bool(signs.get("invert_roll",  False))
invert_pitch = bool(signs.get("invert_pitch", False))

mix = cfg.get("mix", {"pitch_to_ankle":0.9, "roll_to_ankle":0.9})
a_pitch = float(mix.get("pitch_to_ankle", 0.9))
a_roll  = float(mix.get("roll_to_ankle",  0.9))


# --- Targets (objetivos) en grados desde customData ---
targets_deg = cfg.get("targets_deg", {"roll": 0.0, "pitch": 0.0})
t_r = math.radians(float(targets_deg.get("roll", 0.0)))   # objetivo roll (rad)
t_p = math.radians(float(targets_deg.get("pitch", 0.0)))  # objetivo pitch (rad)



limits = cfg.get("limits", {})
ankle_pos_lim = float(limits.get("ankle_pos", 0.20))
ankle_vel_lim = float(limits.get("ankle_vel", 2.0))
hip_pos_lim   = float(limits.get("hip_pos",   0.15))
hip_vel_lim   = float(limits.get("hip_vel",   1.5))

mapping = cfg.get("mapping", {"ankle_roll_opposite": True, "hip_roll_opposite": False})
ankle_roll_opposite = bool(mapping.get("ankle_roll_opposite", True))
hip_roll_opposite   = bool(mapping.get("hip_roll_opposite",   False))

jmap = cfg.get("joints", {})

# 3) Dispositivos (usar getDevice para evitar deprecation)
def get_motor(name: str, vel_limit: float):
    if not name:
        return None
    m = robot.getDevice(name)  # <- NUEVO
    if m:
        m.setVelocity(vel_limit)
    return m

# Sensores con getDevice()
imu = robot.getDevice(imu_name) if imu_name else None
if imu: imu.enable(ts)
gyro = robot.getDevice(gyro_name) if gyro_name else None
if gyro: gyro.enable(ts)

# Motores
mot = {
    "L": {
        "ankle_pitch": get_motor(jmap.get("ankle_pitch_left"),  ankle_vel_lim),
        "ankle_roll":  get_motor(jmap.get("ankle_roll_left"),   ankle_vel_lim),
        "hip_pitch":   get_motor(jmap.get("hip_pitch_left"),    hip_vel_lim),
        "hip_roll":    get_motor(jmap.get("hip_roll_left"),     hip_vel_lim),
    },
    "R": {
        "ankle_pitch": get_motor(jmap.get("ankle_pitch_right"), ankle_vel_lim),
        "ankle_roll":  get_motor(jmap.get("ankle_roll_right"),  ankle_vel_lim),
        "hip_pitch":   get_motor(jmap.get("hip_pitch_right"),   hip_vel_lim),
        "hip_roll":    get_motor(jmap.get("hip_roll_right"),    hip_vel_lim),
    }
}

# Chequeo de motores
missing = []
for side in ("L","R"):
    for j in ("ankle_pitch","ankle_roll","hip_pitch","hip_roll"):
        if mot[side][j] is None:
            missing.append(f"{side}.{j}")
if missing:
    print("[ERR] Motores no encontrados:", ", ".join(missing))
else:
    print("[OK] Todos los motores encontrados.")

# 4) Calibración de offsets (promedia 1.0 s)
calib_s = 1.0
calib_steps = max(1, int((1000.0/ts) * calib_s))
calib_sum_r = calib_sum_p = 0.0
calib_n = 0
r0 = p0 = 0.0

start_time = robot.getTime()

# 5) Rate limiting (AHORA que ya existe 'ts')
rate_limit = math.radians(20.0) * (ts/1000.0)  # 15 deg/s
last_cmds = {"ankle_pitch":0.0, "hip_pitch":0.0, "ankle_roll":0.0, "hip_roll":0.0}

def soft_scale():
    if soft_start_s <= 0: return 1.0
    return clamp((robot.getTime() - start_time) / soft_start_s, 0.0, 1.0)

def apply_pair_same(lm, rm, cmd, pos_lim, key):
    cmd = clamp(cmd, last_cmds[key] - rate_limit, last_cmds[key] + rate_limit)
    last_cmds[key] = cmd
    c = clamp(cmd, -pos_lim, pos_lim)
    if lm: lm.setPosition(c)
    if rm: rm.setPosition(c)

def apply_pair_opposite(lm, rm, cmd, pos_lim, key):
    cmd = clamp(cmd, last_cmds[key] - rate_limit, last_cmds[key] + rate_limit)
    last_cmds[key] = cmd
    c = clamp(cmd, -pos_lim, pos_lim)
    if lm: lm.setPosition(+c)
    if rm: rm.setPosition(-c)

# Guardas de caída
fall_pitch_deg = 25.0
fall_roll_deg  = 25.0
fall_time_s    = 0.30
fall_counter = 0
fall_limit_steps = int((1000.0/ts) * fall_time_s)

divider = max(1, int((1000/ts) / max(1, rate_hz)))
i = 0

print(f"[INFO] imu={imu_name} gyro={gyro_name} rate={rate_hz}Hz | gains R(kp={g_roll['kp']},kd={g_roll['kd']}), P(kp={g_pitch['kp']},kd={g_pitch['kd']})")

# -------------------- Bucle --------------------
while robot.step(ts) != -1:
    if not imu:
        continue

    roll, pitch, _ = imu.getRollPitchYaw()
    wx = wy = 0.0
    if gyro:
        wx, wy, _ = gyro.getValues()

    # Calibración (promedio inicial)
    if calib_n < calib_steps:
        calib_sum_r += roll
        calib_sum_p += pitch
        calib_n += 1
        r0 = calib_sum_r / calib_n
        p0 = calib_sum_p / calib_n

    # Errores con offsets Y objetivos (targets)
    e_r_raw = (roll  - r0) - t_r   # roll hacia el objetivo 
    e_p_raw = (pitch - p0) - t_p   # pitch hacia el objetivo
    e_r = -e_r_raw if invert_roll  else e_r_raw
    e_p = -e_p_raw if invert_pitch else e_p_raw


    # Deadband
    if abs(e_r) < deadband: e_r = 0.0
    if abs(e_p) < deadband: e_p = 0.0

    # PD (signo negativo → empujar contra el error)
    u_r = -(g_roll["kp"]  * e_r)  - (g_roll["kd"]  * wx if gyro else 0.0)
    u_p = -(g_pitch["kp"] * e_p)  - (g_pitch["kd"] * wy if gyro else 0.0)

    s = soft_scale()

    # Mezcla tobillo/cadera
    ankle_pitch_cmd = s * (a_pitch       * u_p)
    hip_pitch_cmd   = s * ((1.0-a_pitch) * u_p)

    ankle_roll_cmd  = s * (a_roll        * u_r)
    hip_roll_cmd    = s * ((1.0-a_roll)  * u_r)

    # Aplicación según mapeo
    apply_pair_same(mot["L"]["ankle_pitch"], mot["R"]["ankle_pitch"], ankle_pitch_cmd, ankle_pos_lim, "ankle_pitch")
    apply_pair_same(mot["L"]["hip_pitch"],   mot["R"]["hip_pitch"],   hip_pitch_cmd,   hip_pos_lim,   "hip_pitch")

    if ankle_roll_opposite:
        apply_pair_opposite(mot["L"]["ankle_roll"], mot["R"]["ankle_roll"], ankle_roll_cmd, ankle_pos_lim, "ankle_roll")
    else:
        apply_pair_same(    mot["L"]["ankle_roll"], mot["R"]["ankle_roll"], ankle_roll_cmd, ankle_pos_lim, "ankle_roll")

    if hip_roll_opposite:
        apply_pair_opposite(mot["L"]["hip_roll"],   mot["R"]["hip_roll"],   hip_roll_cmd,   hip_pos_lim,   "hip_roll")
    else:
        apply_pair_same(    mot["L"]["hip_roll"],   mot["R"]["hip_roll"],   hip_roll_cmd,   hip_pos_lim,   "hip_roll")

    # Guardas de caída
    if abs(math.degrees(e_r_raw)) > fall_roll_deg or abs(math.degrees(e_p_raw)) > fall_pitch_deg:
        fall_counter += 1
        if fall_counter >= fall_limit_steps:
            for side in ("L","R"):
                for j in ("ankle_pitch","ankle_roll","hip_pitch","hip_roll"):
                    if mot[side][j]: mot[side][j].setPosition(0.0)
            print("[SAFE] caída detectada → relajo mandos")
            fall_counter = 0
    else:
        fall_counter = 0

    # Logs
    if DEBUG and i % divider == 0:
        print(f"[SENSE] roll={roll:+.3f} pitch={pitch:+.3f} wx={wx:+.3f} wy={wy:+.3f} | off r0={r0:+.3f} p0={p0:+.3f}")
        print(f"[CMD]   aP={ankle_pitch_cmd:+.3f} hP={hip_pitch_cmd:+.3f} | aR={ankle_roll_cmd:+.3f} hR={hip_roll_cmd:+.3f} | s={s:.2f}")
    i += 1
