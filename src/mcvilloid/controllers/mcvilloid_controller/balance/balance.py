# src/mcvilloid/controllers/mcvilloid_controller/balance/balance.py
import math
from .filters import Lowpass, deadband_deg, SoftStart

class BalanceController:
    """
    Encapsula lazo de balance (pitch/roll) y mezcla a articulaciones.
    Usa params['gains'], ['ki'], ['mix'], ['limits'], ['signs'], ['joint_signs'],
    ['fc_d_pitch_hz'], ['fc_d_roll_hz'], ['deadband_deg'], ['soft_start_s'].
    """
    def __init__(self, params: dict, logger=print):
        self.p = params or {}
        self.logger = logger

        self.enable_I = bool(self.p.get("enable_I", False))
        self.ki = self.p.get("ki", {"pitch": 0.0, "roll": 0.0})
        self.gains = self.p.get("gains", {})
        self.mix = self.p.get("mix", {})
        self.signs = self.p.get("signs", {"invert_pitch": False, "invert_roll": False})
        self.joint_signs = self.p.get("joint_signs", {})
        self.limits = self.p.get("limits", {})
        self.deadband = float(self.p.get("deadband_deg", 0.0))
        self.softstart = SoftStart(self.p.get("soft_start_s", 0.0))

        # filtros derivativos por eje
        self._lp_d_pitch = None
        self._lp_d_roll = None

        # integradores
        self.i_pitch = 0.0
        self.i_roll = 0.0

        # cache de nombres de joints (ajusta a tu convención)
        self.joints = {
            "ankle_pitch": "LLegUay",  # se multiplicará por sign específico
            "hip_pitch":   "LLegLhy",
            "ankle_roll":  "LLegLax",
            "hip_roll":    "LLegMhx",
            # Para la pierna derecha usa mapping en el controller principal si lo requieres
        }

    def _ensure_filters(self, dt: float):
        fc_p = float(self.p.get("fc_d_pitch_hz", 20.0))
        fc_r = float(self.p.get("fc_d_roll_hz", 20.0))
        if self._lp_d_pitch is None:
            self._lp_d_pitch = Lowpass(fc_p, dt)
        else:
            self._lp_d_pitch.set(fc_p, dt)
        if self._lp_d_roll is None:
            self._lp_d_roll = Lowpass(fc_r, dt)
        else:
            self._lp_d_roll.set(fc_r, dt)

    def reset(self):
        self.i_pitch = 0.0
        self.i_roll  = 0.0
        self.softstart.reset()

    def step(self, dt: float, imu_pitch_rad: float, imu_roll_rad: float) -> dict:
        """
        Devuelve offsets dict {joint_name: delta_pos_rad} a sumar a targets base.
        """
        self._ensure_filters(dt)
        g = self.softstart.gain(dt)

        # error en grados (más intuitivo para deadband)
        err_p_deg = math.degrees(imu_pitch_rad)
        err_r_deg = math.degrees(imu_roll_rad)

        # invertir signos si aplica
        if self.signs.get("invert_pitch", False):
            err_p_deg = -err_p_deg
        if self.signs.get("invert_roll", False):
            err_r_deg = -err_r_deg

        # deadband
        e_p_db = deadband_deg(err_p_deg, self.deadband)
        e_r_db = deadband_deg(err_r_deg, self.deadband)

        # derivadas filtradas (en deg/s -> convertir a rad/s al final)
        d_p = self._lp_d_pitch.step((e_p_db - getattr(self, "_prev_p", 0.0)) / max(dt,1e-6))
        d_r = self._lp_d_roll.step((e_r_db - getattr(self, "_prev_r", 0.0)) / max(dt,1e-6))
        self._prev_p = e_p_db
        self._prev_r = e_r_db

        # integrar si habilitado (en deg*s)
        if self.enable_I:
            self.i_pitch += e_p_db * dt
            self.i_roll  += e_r_db * dt

        # PID (realmente PD + I opcional) en grados -> luego pasamos a rad
        kp_p = float(self.gains.get("pitch", {}).get("kp", 0.0))
        kd_p = float(self.gains.get("pitch", {}).get("kd", 0.0))
        kp_r = float(self.gains.get("roll",  {}).get("kp", 0.0))
        kd_r = float(self.gains.get("roll",  {}).get("kd", 0.0))
        ki_p = float(self.ki.get("pitch", 0.0)) if self.enable_I else 0.0
        ki_r = float(self.ki.get("roll",  0.0)) if self.enable_I else 0.0

        u_p_deg = kp_p * e_p_db + kd_p * d_p + ki_p * self.i_pitch
        u_r_deg = kp_r * e_r_db + kd_r * d_r + ki_r * self.i_roll

        # pasar a rad
        u_p = math.radians(u_p_deg) * g
        u_r = math.radians(u_r_deg) * g

        # mezclar a tobillo/cadera
        mix_pa = float(self.mix.get("pitch_to_ankle", 0.5))  # 0..1
        mix_ra = float(self.mix.get("roll_to_ankle",  0.5))

        u_ap = u_p * mix_pa
        u_hp = u_p * (1.0 - mix_pa)
        u_ar = u_r * mix_ra
        u_hr = u_r * (1.0 - mix_ra)

        # aplicar signos por joint (para coincidir dirección real)
        s_ap = float(self.joint_signs.get("ankle_pitch", 1.0))
        s_hp = float(self.joint_signs.get("hip_pitch",   1.0))
        s_ar = float(self.joint_signs.get("ankle_roll",  1.0))
        s_hr = float(self.joint_signs.get("hip_roll",    1.0))

        deltas = {
            "LLegUay": s_ap * u_ap,   # ankle pitch L
            "LLegLhy": s_hp * u_hp,   # hip pitch L
            "LLegLax": s_ar * u_ar,   # ankle roll L
            "LLegMhx": s_hr * u_hr,   # hip roll L
            "RLegUay": s_ap * u_ap,   # ankle pitch R (si comparten signo)
            "RLegLhy": s_hp * u_hp,
            "RLegLax": s_ar * u_ar,
            "RLegMhx": s_hr * u_hr,
        }

        # clamp por límites (pos/vel) se hace fuera, sobre target final
        return deltas
