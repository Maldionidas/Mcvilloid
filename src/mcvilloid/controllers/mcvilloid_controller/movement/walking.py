# src/mcvilloid/controllers/mcvilloid_controller/movement/walking.py
import math

class Walker:
    def __init__(self, params, limits, logger=print):
        self.logger = logger
        self.limits = limits or {}

        # Config de marcha
        self.cfg = (params.get("gait") or {})
        gcfg = self.cfg
        # ---- Utilidades ----
        def _clamp(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)
        self._clamp = _clamp

        dir_str = (gcfg.get("direction") or "forward").lower()
        self.dir = 1.0 if dir_str in ("fwd", "forward", "+", "1", "+1") else -1.0

        # ---- Flags básicos (sin duplicados) ----
        self.enabled = bool(gcfg.get("auto_on", False))
        self.on = self.enabled
        self.freq_hz = float(gcfg.get("freq_hz", 0.5))
        self.soft_start_s = float(gcfg.get("soft_start_s", 0.5))
        self.t_since_enable = 0.0
        self._global_gain = 1.0

        # Robustez: límites razonables
        self.freq_hz = self._clamp(self.freq_hz, 0.05, 2.0)


        # Fase y offsets de pierna
        ph_off = gcfg.get("phase_offset") or {}
        self.phase = 0.0
        self.phase_off_L = float(ph_off.get("L", 0.0))
        self.phase_off_R = float(ph_off.get("R", math.pi))

        # Amplitudes de paso (pitch)
        step_amp = gcfg.get("step_amp") or {}
        self.hip_pitch_A   = float(step_amp.get("hip_pitch",   0.20))
        self.knee_pitch_A  = float(step_amp.get("knee_pitch",  0.45))
        self.ankle_pitch_A = float(step_amp.get("ankle_pitch", 0.20))

        # Amplitudes laterales (roll)
        lat_amp = gcfg.get("lateral_amp") or {}
        self.hip_roll_A    = float(lat_amp.get("hip_roll",   0.06))
        self.ankle_roll_A  = float(lat_amp.get("ankle_roll", 0.05))

        # Bias (sesgos estáticos)
        bias = gcfg.get("bias") or {}
        self.bias_torso_pitch = float(bias.get("torso_pitch", 0.0))
        self.bias_ankle_pitch = float(bias.get("ankle_pitch", 0.0))

        # Logging
        self.last_phase_logged = -1.0

        # Mapeo de nombres de articulaciones (con tu PROTO)
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

    def set_global_gain(self, g: float):
        # clamp 0..1
        self._global_gain = 0.0 if g < 0 else (1.0 if g > 1.0 else g)

    def toggle(self, on: bool):
        if on and not self.enabled:
            self.t_since_enable = 0.0
        self.enabled = on
        self.on = on

    def _soft_gain(self):
        """Ramp suave tipo S para arranque."""
        if self.soft_start_s <= 0:
            return 1.0
        x = max(0.0, min(1.0, self.t_since_enable / self.soft_start_s))
        return x * x * (3.0 - 2.0 * x)

    def step(self, dt, imu, _unused):
        """Genera offsets de marcha (diccionario joint->offset)."""
        if not self.enabled:
            return {}

        self.t_since_enable += dt
        self.phase = (self.phase + math.tau * self.freq_hz * dt) % math.tau

        def sinus(A, phase, off=0.0):
            return (A * self._global_gain) * math.sin(phase + off)

        # Fases por pierna
        phiL = self.phase + self.phase_off_L
        phiR = self.phase + self.phase_off_R

        # Ganancia de arranque
        g = self._soft_gain()

        kps = self.dir * (math.pi/2)

        # --- Pierna izquierda (solo PITCH lleva self.dir; ROLL no) ---
        hip_pitch_L   = g * self.dir * self.hip_pitch_A   * math.sin(phiL)              + self.bias_torso_pitch
        knee_pitch_L  = g * self.dir * self.knee_pitch_A  * math.sin(phiL + kps)
        ankle_pitch_L = g * self.dir * self.ankle_pitch_A * math.sin(phiL)              + self.bias_ankle_pitch
        hip_roll_L    = g *              self.hip_roll_A   * math.sin(phiL + math.pi/2)
        ankle_roll_L  = g *              self.ankle_roll_A * math.sin(phiL)

        # --- Pierna derecha ---
        hip_pitch_R   = g * self.dir * self.hip_pitch_A   * math.sin(phiR)              + self.bias_torso_pitch
        knee_pitch_R  = g * self.dir * self.knee_pitch_A  * math.sin(phiR + kps)
        ankle_pitch_R = g * self.dir * self.ankle_pitch_A * math.sin(phiR)              + self.bias_ankle_pitch
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
                f"bias(tp,ap)={self.bias_torso_pitch:.2f},{self.bias_ankle_pitch:.2f}"
                f"g={self._global_gain:.2f}"
            )
            self.last_phase_logged = self.phase

        return out

    # Alias para compatibilidad con el controller
    def update(self, dt, imu, cmds):
        return self.step(dt, imu, cmds)
