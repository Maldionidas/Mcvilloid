# src/mcvilloid/controllers/mcvilloid_controller/movement/walking.py
import math

class Walker:
    def __init__(self, params, limits, logger=print):
        self.logger = logger
        self.limits = limits or {}

        # ---- Config de marcha: acepta params = {...} o params = {"gait": {...}} ----
        root = params or {}
        gcfg = root.get("gait", root)

        self.enabled = bool(gcfg.get("auto_on", False))
        self.on = self.enabled

        # Dirección (solo afecta PITCH)
        dir_str = (gcfg.get("direction") or "forward").lower()
        self.dir = 1.0 if dir_str in ("fwd", "forward", "+", "1", "+1") else -1.0

        # Frecuencia y arranque suave
        self.freq_hz = float(gcfg.get("freq_hz", 0.33))
        self.soft_start_s = float(gcfg.get("soft_start_s", 1.2))
        self.freq_hz = self._clamp(self.freq_hz, 0.05, 2.0)
        self.t_since_enable = 0.0
        self._global_gain = 1.0

        # Fase
        ph_off = gcfg.get("phase_offset") or {}
        self.phase = 0.0
        self.phase_off_L = float(ph_off.get("L", 0.0))
        self.phase_off_R = float(ph_off.get("R", math.pi))  # 180° out of phase

        # Amplitudes de paso (pitch)
        step_amp = gcfg.get("step_amp") or {}
        self.hip_pitch_A   = float(step_amp.get("hip_pitch",   0.13))
        self.knee_pitch_A  = float(step_amp.get("knee_pitch",  0.20))
        self.ankle_pitch_A = float(step_amp.get("ankle_pitch", 0.02))  # un poco menor para no rozar clamp

        # Amplitudes laterales (roll) — pequeñitas
        lat_amp = gcfg.get("lateral_amp") or {}
        self.hip_roll_A    = float(lat_amp.get("hip_roll",   0.00))
        self.ankle_roll_A  = float(lat_amp.get("ankle_roll", 0.01))

        # Sesgos estáticos
        bias = gcfg.get("bias") or {}
        self.bias_torso_pitch = float(bias.get("torso_pitch", 0.10))  # empuja COM adelante
        self.bias_ankle_pitch = float(bias.get("ankle_pitch", 0.00))

        # Mapeo de nombres de articulaciones (de tu PROTO)
        self.j = {
            "L": {
                "hip_pitch":   "j00_hip_pitch_l",
                "hip_roll":    "j01_hip_roll_l",
                "hip_yaw":     "j02_hip_yaw_l",
                "knee_pitch":  "j03_knee_pitch_l",
                "ankle_pitch": "j04_ankle_pitch_l",
                "ankle_roll":  "j05_ankle_roll_l",
            },
            "R": {
                "hip_pitch":   "j06_hip_pitch_r",
                "hip_roll":    "j07_hip_roll_r",
                "hip_yaw":     "j08_hip_yaw_r",
                "knee_pitch":  "j09_knee_pitch_r",
                "ankle_pitch": "j10_ankle_pitch_r",
                "ankle_roll":  "j11_ankle_roll_r",
            },
        }

        self.last_phase_logged = -1.0

    # ---------------------------- Utils ----------------------------
    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def _soft_gain(self):
        """Ramp suave tipo S para arranque."""
        if self.soft_start_s <= 0:
            return 1.0
        x = max(0.0, min(1.0, self.t_since_enable / self.soft_start_s))
        return x * x * (3.0 - 2.0 * x)

    def set_global_gain(self, g: float):
        self._global_gain = 0.0 if g < 0 else (1.0 if g > 1.0 else g)

    def toggle(self, on: bool):
        if on and not self.enabled:
            self.t_since_enable = 0.0
        self.enabled = on
        self.on = on

    # ----------------------------- Gait -----------------------------
    def step(self, dt, imu, _unused):
        """
        Genera offsets de marcha (dict joint->offset en rad).
        - Cadera y rodilla en fase (avance).
        - Tobillo en contrafase suave (despegue/aterrizaje).
        - Roll mínimo para evitar recover por lateral.
        """
        if not self.enabled:
            return {}

        self.t_since_enable += dt
        self.phase = (self.phase + math.tau * self.freq_hz * dt) % math.tau

        # Fases por pierna
        phiL = self.phase + self.phase_off_L
        phiR = self.phase + self.phase_off_R

        # --- Guardas anti-caída (backfall) ---
        back_thr = 0.10     # empieza ayuda si pitch < -0.10 rad
        back_max = 0.30     # saturación de ayuda
        p = getattr(imu, "pitch", 0.0)

        extra_tp = 0.0
        amp_scale = 1.0
        if p < -back_thr:
            d = min(back_max, (-p - back_thr))
            # empuja el torso más al frente (DC bias en cadera)
            extra_tp = 0.35 * d          # ~0..0.07 rad típico
            # baja amplitud de paso para recuperar
            amp_scale = max(0.5, 1.0 - 1.8 * d)

        # aplica escala de emergencia a amplitudes
        HpA   = self.hip_pitch_A   * amp_scale
        KnA   = self.knee_pitch_A  * amp_scale
        AnkA  = self.ankle_pitch_A * amp_scale
        tp_bias = self.bias_torso_pitch + extra_tp

        # Ganancias
        g = self._soft_gain() * self._global_gain
        kps = self.dir * (math.pi / 2.0)  # desfase de 90° para la rodilla respecto a cadera

        # --- PITCH (avanza solo con dir) ---
        hip_pitch_L   = g * self.dir * HpA  * math.sin(phiL) + tp_bias
        knee_pitch_L  = g * self.dir * KnA  * math.sin(phiL + kps)
        ankle_pitch_L = g * self.dir * AnkA * math.sin(phiL) + self.bias_ankle_pitch

        hip_pitch_R   = g * self.dir * HpA  * math.sin(phiR) + tp_bias
        knee_pitch_R  = g * self.dir * KnA  * math.sin(phiR + kps)
        ankle_pitch_R = g * self.dir * AnkA * math.sin(phiR) + self.bias_ankle_pitch

        # --- ROLL (muy pequeño y sin dir) ---
        hip_roll_L    = g *              self.hip_roll_A   * math.sin(phiL + math.pi/2)
        ankle_roll_L  = g *              self.ankle_roll_A * math.sin(phiL)
        hip_roll_R    = g *              self.hip_roll_A   * math.sin(phiR + math.pi/2)
        ankle_roll_R  = g *              self.ankle_roll_A * math.sin(phiR)

        out = {
            self.j["L"]["hip_pitch"]:   hip_pitch_L,
            self.j["L"]["knee_pitch"]:  knee_pitch_L,
            self.j["L"]["ankle_pitch"]: ankle_pitch_L,
            self.j["L"]["hip_roll"]:    hip_roll_L,
            self.j["L"]["ankle_roll"]:  ankle_roll_L,

            self.j["R"]["hip_pitch"]:   hip_pitch_R,
            self.j["R"]["knee_pitch"]:  knee_pitch_R,
            self.j["R"]["ankle_pitch"]: ankle_pitch_R,
            self.j["R"]["hip_roll"]:    hip_roll_R,
            self.j["R"]["ankle_roll"]:  ankle_roll_R,
        }

        # Log compacto cada ~0.25 rad
        if self.last_phase_logged < 0 or abs(self.phase - self.last_phase_logged) > 0.25:
            self.logger(
                f"[gait] phase={self.phase:.2f} Δ={len(out)} joints | "
                f"A(hp,kn,ap)={self.hip_pitch_A:.2f},{self.knee_pitch_A:.2f},{self.ankle_pitch_A:.2f} "
                f"bias(tp,ap)={self.bias_torso_pitch:.2f},{self.bias_ankle_pitch:.2f} "
                f"g={self._global_gain:.2f}"
            )
            self.last_phase_logged = self.phase

        # Clamps por articulación con margen seguro
        def _lim(name, val):
            lim = (self.limits.get(name) or {})
            lo, hi = lim.get("min", -9), lim.get("max", 9)
            return self._clamp(val, lo, hi)

        out[self.j["L"]["hip_pitch"]]   = _lim("hip_pitch_cmd",   out[self.j["L"]["hip_pitch"]])
        out[self.j["L"]["knee_pitch"]]  = _lim("knee_pitch_cmd",  out[self.j["L"]["knee_pitch"]])
        out[self.j["L"]["ankle_pitch"]] = _lim("ankle_pitch_cmd", out[self.j["L"]["ankle_pitch"]])

        out[self.j["R"]["hip_pitch"]]   = _lim("hip_pitch_cmd",   out[self.j["R"]["hip_pitch"]])
        out[self.j["R"]["knee_pitch"]]  = _lim("knee_pitch_cmd",  out[self.j["R"]["knee_pitch"]])
        out[self.j["R"]["ankle_pitch"]] = _lim("ankle_pitch_cmd", out[self.j["R"]["ankle_pitch"]])

        return out

    # Alias para compat con el controller
    def update(self, dt, imu, cmds):
        return self.step(dt, imu, cmds)
