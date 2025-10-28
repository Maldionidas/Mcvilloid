"""
Bípedo - estabilización por juntas (v2.1 → v2.1.1 PID, anotado)
Cambios clave respecto a v2.1:
- (✅) PID real: se usa el integrador i_p/i_r en u_p/u_r (antes era PD).
- (✅) Back-calculation + freeze del I cuando hay saturación fuerte.
- (✅) Control a rate_hz real con divider; dt_ctrl coherente para LPF e integración.
- (✅) Limpieza de gains/config (no sobrescribir g_pitch a destiempo).
- (✅) Seed del LPF gyro en i==0.
- (✅) Reset seguro de last_cmds tras caída.
- (☑️) Pequeña rampa para pitch_ref (opcional, activada aquí con límites suaves).
"""

# ====================== PASO 0: IMPORTS & HELPERS ======================
from controller import Robot
import json, math, os

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

DEBUG = True  # logs extra al arrancar

# ====================== PASO 1: ARRANQUE & CONFIG ======================
print("[BOOT] mcvilloid_controller arrancó")
robot = Robot()

# 1.1) timestep primero (ts se usa por todos lados)
ts = int(robot.getBasicTimeStep()) or 16  # ms
base_dt = ts / 1000.0

# 1.2) lee configuración desde customData (JSON UNA línea en .wbt)
try:
    cfg = json.loads(robot.getCustomData() or "{}")
except Exception:
    print("[WARN] customData inválido; usando defaults.")
    cfg = {}

def deep_update(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_update(dst[k], v)
        else:
            dst[k] = v

params_path = os.path.join(os.path.dirname(__file__), "params.json")
if os.path.exists(params_path):
    try:
        with open(params_path, "r", encoding="utf-8") as f:
            file_cfg = json.load(f)
        deep_update(cfg, file_cfg)
        print("[OK] params.json cargado (override de configuración).")
    except Exception as e:
        print(f"[WARN] params.json inválido: {e}")

# Flag de configuración y compuerta runtime para el integral
cfg_enable_I = bool(cfg.get("enable_I", False))

# 1.3) parámetros (todos ajustables desde customData)
imu_name = cfg.get("imu_name", "imu_sensor")
gyro_name = cfg.get("gyro_name", "gyro")
rate_hz = int(cfg.get("rate_hz", 100))

ctrl_hz = 1000.0 / ts
divider = max(1, int(ctrl_hz / max(1, rate_hz)))

# Objetivos
targets_deg = cfg.get("targets_deg", {"roll": 0.0, "pitch": 0.0})
t_r = math.radians(float(targets_deg.get("roll", 0.0)))   # objetivo roll (rad)
t_p = math.radians(float(targets_deg.get("pitch", 0.0)))  # objetivo pitch (rad)

# Gains: tomar de cfg o defaults de forma coherente
def _mk_gains(block, kp_def, kd_def):
    g = cfg.get("gains", {}).get(block, {"kp": kp_def, "kd": kd_def})
    return {"kp": float(g.get("kp", kp_def)), "kd": float(g.get("kd", kd_def))}

g_roll  = _mk_gains("roll",  1.2, 0.20)
g_pitch = _mk_gains("pitch", 1.2, 0.20)

# Integral
ki_cfg = cfg.get("ki", {"roll": 0.08, "pitch": 0.15})
ki_r = float(ki_cfg.get("roll", 0.08))
ki_p = float(ki_cfg.get("pitch", 0.15))
i_r = 0.0
i_p = 0.0
i_limit = math.radians(2.0)  # máx. 2°

# zona muerta y soft-start
deadband = math.radians(float(cfg.get("deadband_deg", 0.5)))
soft_start_s = float(cfg.get("soft_start_s", 1.5))

# signos globales
a_signs = cfg.get("signs", {"invert_roll": False, "invert_pitch": False})
invert_roll = bool(a_signs.get("invert_roll", False))
invert_pitch = bool(a_signs.get("invert_pitch", False))

# mezcla tobillo/cadera
mix = cfg.get("mix", {"pitch_to_ankle": 0.6, "roll_to_ankle": 0.9})
a_pitch = float(mix.get("pitch_to_ankle", 0.6))  # % del control de pitch al tobillo
a_roll = float(mix.get("roll_to_ankle", 0.9))   # % del control de roll al tobillo

# límites
limits = cfg.get("limits", {})
ankle_pos_lim = float(limits.get("ankle_pos", 0.08))
ankle_vel_lim = float(limits.get("ankle_vel", 2.0))
hip_pos_lim   = float(limits.get("hip_pos", 0.10))
hip_vel_lim   = float(limits.get("hip_vel", 1.5))

# mapping
mapping = cfg.get("mapping", {"ankle_roll_opposite": True, "hip_roll_opposite": False})
ankle_roll_opposite = bool(mapping.get("ankle_roll_opposite", True))
hip_roll_opposite   = bool(mapping.get("hip_roll_opposite", False))

# joints
jmap = cfg.get("joints", {})
joint_signs = cfg.get(
    "joint_signs",
    {"ankle_pitch": 1.0, "hip_pitch": 1.0, "ankle_roll": 1.0, "hip_roll": 1.0},
)
for k in ("ankle_pitch", "hip_pitch", "ankle_roll", "hip_roll"):
    joint_signs[k] = float(joint_signs.get(k, 1.0))

# postura base (bias)
stance = cfg.get("stance", {})
ANKLE_BIAS = math.radians(float(stance.get("ankle_bias_deg", 0.0)))
HIP_BIAS   = math.radians(float(stance.get("hip_bias_deg", 0.0)))

# ====================== PASO 2: DISPOSITIVOS ===========================
def get_motor(name: str, vel_limit: float):
    if not name:
        return None
    m = robot.getDevice(name)  # API moderna
    if m:
        try:
            m.setVelocity(vel_limit)
        except Exception:
            pass
    return m

imu = robot.getDevice(imu_name) if imu_name else None
if imu:
    imu.enable(ts)

gyro = robot.getDevice(gyro_name) if gyro_name else None
if gyro:
    gyro.enable(ts)

mot = {
    "L": {
        "ankle_pitch": get_motor(jmap.get("ankle_pitch_left"), ankle_vel_lim),
        "ankle_roll":  get_motor(jmap.get("ankle_roll_left"),  ankle_vel_lim),
        "hip_pitch":   get_motor(jmap.get("hip_pitch_left"),   hip_vel_lim),
        "hip_roll":    get_motor(jmap.get("hip_roll_left"),    hip_vel_lim),
    },
    "R": {
        "ankle_pitch": get_motor(jmap.get("ankle_pitch_right"), ankle_vel_lim),
        "ankle_roll":  get_motor(jmap.get("ankle_roll_right"),  ankle_vel_lim),
        "hip_pitch":   get_motor(jmap.get("hip_pitch_right"),   hip_vel_lim),
        "hip_roll":    get_motor(jmap.get("hip_roll_right"),    hip_vel_lim),
    },
}

missing = []
for side in ("L", "R"):
    for j in ("ankle_pitch", "ankle_roll", "hip_pitch", "hip_roll"):
        if mot[side][j] is None:
            missing.append(f"{side}.{j}")
print(
    "[OK] Todos los motores encontrados." if not missing else "[ERR] Falta(n): " + ", ".join(missing)
)

# ====================== PASO 3: CALIBRACIÓN & LIMITES ==================
# 3.1) offsets de IMU (promedio ~1s)
calib_s = 1.0
calib_steps = max(1, int((1000.0 / ts) * calib_s))
calib_sum_r = calib_sum_p = 0.0
calib_n = 0
r0 = p0 = 0.0

# 3.2) rate limiting y soft-start
start_time = robot.getTime()
rate_limit = math.radians(60.0) * base_dt  # 60 deg/s — ajustable
last_cmds = {"ankle_pitch": 0.0, "hip_pitch": 0.0, "ankle_roll": 0.0, "hip_roll": 0.0}

def soft_scale():
    if soft_start_s <= 0:
        return 1.0
    return clamp((robot.getTime() - start_time) / soft_start_s, 0.0, 1.0)

def apply_pair_same(lm, rm, cmd, pos_lim, key):
    # rate limiting
    cmd_rl = clamp(cmd, last_cmds[key] - rate_limit, last_cmds[key] + rate_limit)
    last_cmds[key] = cmd_rl
    # position clamp
    c_applied = clamp(cmd_rl, -pos_lim, pos_lim)
    if lm: lm.setPosition(c_applied)
    if rm: rm.setPosition(c_applied)
    return c_applied

def apply_pair_opposite(lm, rm, cmd, pos_lim, key):
    cmd_rl = clamp(cmd, last_cmds[key] - rate_limit, last_cmds[key] + rate_limit)
    last_cmds[key] = cmd_rl
    c_applied = clamp(cmd_rl, -pos_lim, pos_lim)
    if lm: lm.setPosition(+c_applied)
    if rm: rm.setPosition(-c_applied)
    return c_applied

# 3.3) guardas de caída
fall_pitch_deg = 25.0
fall_roll_deg  = 25.0
fall_time_s    = 0.30
fall_counter = 0
fall_limit_steps = int((1000.0 / ts) * fall_time_s)
fallen = False   # latch de caída

# 3.5) derivativo filtrado (LPF 1er orden) para wy (pitch) y wx (roll)
fc_d_pitch = float(cfg.get("fc_d_pitch_hz", 25.0))   # 20–30 Hz suele ir bien a 100 Hz
fc_d_roll  = float(cfg.get("fc_d_roll_hz", 25.0))
wy_prev = 0.0
wx_prev = 0.0

# Ref de prueba + rampa suave de ref (anti-step)
pitch_ref = 0.0
prev_pitch_ref = 0.0
ref_slew = math.radians(10.0) * base_dt  # 10°/s máx.

print(
    f"[INFO] imu={imu_name} gyro={gyro_name} rate={rate_hz}Hz | gains R(kp={g_roll['kp']},kd={g_roll['kd']}), P(kp={g_pitch['kp']},kd={g_pitch['kd']})"
)

# ====================== PASO 4: BUCLE PRINCIPAL ========================
i = 0
while robot.step(ts) != -1:
    # tiempo simulado
    t = robot.getTime()

    # referencia de pitch temporal (demo)
    raw_pitch_ref = 0.05 if 2.0 <= t < 2.3 else 0.0  # ~2.9°
    # rampa de referencia
    pitch_ref = clamp(raw_pitch_ref, prev_pitch_ref - ref_slew, prev_pitch_ref + ref_slew)
    prev_pitch_ref = pitch_ref

    # 4.1) leer sensores (si no hay IMU, no hay control)
    if not imu:
        i += 1
        continue

    roll, pitch, _ = imu.getRollPitchYaw()

    # Lee gyro RAW
    if gyro:
        wx_raw, wy_raw, _ = gyro.getValues()
    else:
        wx_raw = wy_raw = 0.0

    # 4.2) calibración de offsets durante la ventana
    if calib_n < calib_steps:
        calib_sum_r += roll
        calib_sum_p += pitch
        calib_n += 1
        r0 = calib_sum_r / calib_n
        p0 = calib_sum_p / calib_n

    # Solo ejecutar el control cada 'divider' pasos
    if (i % divider) != 0:
        i += 1
        continue

    # dt efectivo del control (coherente con rate_hz)
    dt_ctrl = base_dt * divider

    # Seed LPF una vez al inicio para evitar spikes
    if i == 0 and gyro:
        wy_prev, wx_prev = wy_raw, wx_raw

    # LPF de derivativo (usar dt_ctrl)
    alpha_p = dt_ctrl / (dt_ctrl + 1.0 / (2.0 * math.pi * fc_d_pitch))
    alpha_r = dt_ctrl / (dt_ctrl + 1.0 / (2.0 * math.pi * fc_d_roll))
    wy = alpha_p * wy_raw + (1.0 - alpha_p) * wy_prev
    wx = alpha_r * wx_raw + (1.0 - alpha_r) * wx_prev
    wy_prev, wx_prev = wy, wx

    # 4.3) errores hacia el objetivo (targets) con offsets + pitch_ref
    pitch_target = p0 + t_p + pitch_ref
    roll_target  = r0 + t_r

    e_r_raw = roll  - roll_target
    e_p_raw = pitch - pitch_target

    e_r = -e_r_raw if invert_roll  else e_r_raw
    e_p = -e_p_raw if invert_pitch else e_p_raw

    # 4.4) deadband para limpiar microoscilaciones
    if abs(e_r) < deadband:
        e_r = 0.0
    if abs(e_p) < deadband:
        e_p = 0.0

    # 4.5) rampa suave + compuerta del integrador
    s = soft_scale()
    enable_I_runtime = (cfg_enable_I and (t - start_time) > soft_start_s and calib_n >= calib_steps and (not fallen))

    # 4.6) Ley de control PID (I opcional)
    u_p = -(g_pitch["kp"] * e_p) - (g_pitch["kd"] * wy) - (ki_p * i_p if enable_I_runtime else 0.0)
    u_r = -(g_roll["kp"]  * e_r) - (g_roll["kd"]  * wx) - (ki_r * i_r if enable_I_runtime else 0.0)

    # 4.6 bis) mezcla tobillo/cadera con rampa suave
    ankle_pitch_pre = s * (a_pitch * u_p)
    hip_pitch_pre   = s * ((1.0 - a_pitch) * u_p)
    ankle_roll_pre  = s * (a_roll * u_r)
    hip_roll_pre    = s * ((1.0 - a_roll) * u_r)

    # 4.7) signos de cada familia de junta (1 sola vez)
    ankle_pitch_pre *= joint_signs.get("ankle_pitch", 1.0)
    hip_pitch_pre   *= joint_signs.get("hip_pitch",   1.0)
    ankle_roll_pre  *= joint_signs.get("ankle_roll",  1.0)
    hip_roll_pre    *= joint_signs.get("hip_roll",    1.0)

    # 4.8) bias de postura (antes de aplicar a motores)
    ankle_pitch_cmd = ankle_pitch_pre + ANKLE_BIAS
    hip_pitch_cmd   = hip_pitch_pre   + HIP_BIAS
    ankle_roll_cmd  = ankle_roll_pre
    hip_roll_cmd    = hip_roll_pre

    if DEBUG:
        print(
            f"[SIGN] e_p={e_p:+.3f} u_p={u_p:+.3f} "
            f"a_pre={ankle_pitch_pre:+.3f} js_ap={joint_signs.get('ankle_pitch',1.0):+.1f} "
            f"h_pre={hip_pitch_pre:+.3f} js_hp={joint_signs.get('hip_pitch',1.0):+.1f} "
            f"cmd={ankle_pitch_cmd:+.3f}"
        )

    # 4.9) enviar a motores con limitadores
    applied_aP = apply_pair_same(
        mot["L"]["ankle_pitch"], mot["R"]["ankle_pitch"],
        ankle_pitch_cmd, ankle_pos_lim, "ankle_pitch"
    )
    applied_hP = apply_pair_same(
        mot["L"]["hip_pitch"], mot["R"]["hip_pitch"],
        hip_pitch_cmd, hip_pos_lim, "hip_pitch"
    )
    if ankle_roll_opposite:
        applied_aR = apply_pair_opposite(
            mot["L"]["ankle_roll"], mot["R"]["ankle_roll"],
            ankle_roll_cmd, ankle_pos_lim, "ankle_roll"
        )
    else:
        applied_aR = apply_pair_same(
            mot["L"]["ankle_roll"], mot["R"]["ankle_roll"],
            ankle_roll_cmd, ankle_pos_lim, "ankle_roll"
        )

    if hip_roll_opposite:
        applied_hR = apply_pair_opposite(
            mot["L"]["hip_roll"], mot["R"]["hip_roll"],
            hip_roll_cmd, hip_pos_lim, "hip_roll"
        )
    else:
        applied_hR = apply_pair_same(
            mot["L"]["hip_roll"], mot["R"]["hip_roll"],
            hip_roll_cmd, hip_pos_lim, "hip_roll"
        )

    # Para logs y anti-windup
    if abs(e_r) < deadband:
        e_r_P = 0.0
    else:
        e_r_P = e_r
    if abs(e_p) < deadband:
        e_p_P = 0.0
    else:
        e_p_P = e_p

    # === Anti-windup (back-calculation) ===
    # Ganancia de anti-windup (suave): ~ kp/5 relativo a cada lazo
    K_AW_P = float(cfg.get("k_aw_pitch", g_pitch["kp"] / 5.0))
    K_AW_R = float(cfg.get("k_aw_roll",  g_roll["kp"]  / 5.0))

    # Usamos el tobillo como actuador dominante del lazo de pitch/roll:
    # error de saturación = (aplicado - deseado) en la junta principal.
    sat_err_p = (applied_aP - ankle_pitch_cmd)
    sat_err_r = (applied_aR - ankle_roll_cmd)

    if enable_I_runtime and not fallen:
        # Congelar I si saturación fuerte (>25% del rango útil)
        if abs(sat_err_p) <= 0.25 * ankle_pos_lim:
            i_p = clamp(i_p + e_p_P * dt_ctrl + (K_AW_P * sat_err_p) * dt_ctrl, -i_limit, +i_limit)
        if abs(sat_err_r) <= 0.25 * ankle_pos_lim:
            i_r = clamp(i_r + e_r_P * dt_ctrl + (K_AW_R * sat_err_r) * dt_ctrl, -i_limit, +i_limit)

    # 4.10) guardas de caída (seguridad) con latch
    if (abs(math.degrees(e_r_raw)) > fall_roll_deg) or (abs(math.degrees(e_p_raw)) > fall_pitch_deg):
        fall_counter += 1
    else:
        fall_counter = 0

    if fall_counter >= fall_limit_steps and not fallen:
        # suelta mandos y resetea estados del controlador
        for side in ("L","R"):
            for j in ("ankle_pitch","ankle_roll","hip_pitch","hip_roll"):
                if mot[side][j]:
                    mot[side][j].setPosition(0.0)
        i_p = 0.0
        i_r = 0.0
        for k in last_cmds:
            last_cmds[k] = 0.0
        fallen = True
        print("[SAFE] caída detectada → relajo mandos")
        i += 1
        continue

    # Si estamos caídos, mantener neutro hasta que vuelva cerca del centro
    if fallen:
        if abs(math.degrees(pitch - p0)) < 5.0 and abs(math.degrees(roll - r0)) < 5.0:
            fallen = False  # rearmar
        else:
            # mantener postura neutra mientras tanto
            if mot["L"]["ankle_pitch"]: mot["L"]["ankle_pitch"].setPosition(ANKLE_BIAS)
            if mot["R"]["ankle_pitch"]: mot["R"]["ankle_pitch"].setPosition(ANKLE_BIAS)
            if mot["L"]["hip_pitch"]:   mot["L"]["hip_pitch"].setPosition(HIP_BIAS)
            if mot["R"]["hip_pitch"]:   mot["R"]["hip_pitch"].setPosition(HIP_BIAS)
            if mot["L"]["ankle_roll"]:  mot["L"]["ankle_roll"].setPosition(0.0)
            if mot["R"]["ankle_roll"]:  mot["R"]["ankle_roll"].setPosition(0.0)
            if mot["L"]["hip_roll"]:    mot["L"]["hip_roll"].setPosition(0.0)
            if mot["R"]["hip_roll"]:    mot["R"]["hip_roll"].setPosition(0.0)
            i += 1
            continue

    # 4.11) logs de depuración
    if DEBUG:
        print(f"[SENSE] roll={roll:+.3f} pitch={pitch:+.3f} wx={wx:+.3f} wy={wy:+.3f} | off r0={r0:+.3f} p0={p0:+.3f}")
        print(f"[CMD]   aP={applied_aP:+.3f} hP={applied_hP:+.3f} | aR={applied_aR:+.3f} hR={applied_hR:+.3f} | s={s:.2f}")

    i += 1
