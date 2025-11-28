# src/mcvilloid/controllers/mcvilloid_controller/balance/balance.py
import math
from .filters import Lowpass, deadband_deg, SoftStart


class BalanceController:
    """
    Controlador de balance para el robot (pitch y roll).

    - Implementa un control PD (+I opcional) por ejes:
        * pitch: inclinación hacia adelante / atrás
        * roll: inclinación lateral
    - Convierte el error de IMU (rad) a grados para aplicar:
        * deadband (zona muerta)
        * quiet-zone (zona de baja ganancia cerca del equilibrio)
        * filtros derivativos
    - Mezcla las salidas en articulaciones:
        * Tobillos: j04_ankle_pitch_l, j10_ankle_pitch_r, j05_ankle_roll_l, j11_ankle_roll_r
        * Caderas:  j00_hip_pitch_l, j06_hip_pitch_r, j01_hip_roll_l, j07_hip_roll_r

    Lee de params:
      - enable_I: bool, habilita término integral.
      - gains:   { "pitch": {kp, kd}, "roll": {kp, kd} } en unidades de grados.
      - ki:      { "pitch": ki_p, "roll": ki_r } integrales (deg⁻¹ * s⁻¹ aprox).
      - mix:     { "pitch_to_ankle", "roll_to_ankle" } mezcla tobillo/cadera.
      - limits:  { "ankle_cmd": max_rad } límite de comando de tobillo.
      - signs:   { invert_pitch, invert_roll } invierte error de IMU.
      - joint_signs:
            {
                "ankle_pitch_l", "ankle_pitch_r", "ankle_pitch",
                "hip_pitch_l",   "hip_pitch_r",   "hip_pitch",
                "ankle_roll_l",  "ankle_roll_r",  "ankle_roll",
                "hip_roll_l",    "hip_roll_r",    "hip_roll",
            }
      - fc_d_pitch_hz, fc_d_roll_hz: frecuencia de corte de derivadas (Hz).
      - deadband_deg:     zona muerta en grados alrededor de 0.
      - soft_start_s:     tiempo de rampa para subir la ganancia de 0→1.
      - cmd_lpf_tau_s:    constante de tiempo del filtro de salida (τ).
      - quiet_zone_deg:   holgura adicional para bajar Kp y anular D cerca de 0.
      - i_limit_deg:      límite del integrador en deg.
    """

    def __init__(self, params: dict, logger=print):
        self.p = params or {}
        self.logger = logger

        # Flags y ganancias
        self.enable_I = bool(self.p.get("enable_I", False))
        self.gains = self.p.get("gains", {})
        self.ki = self.p.get("ki", {"pitch": 0.0, "roll": 0.0})

        # Mezclas globales (pitch/roll → tobillos y caderas)
        self.mix = self.p.get("mix", {})
        self.signs = self.p.get("signs", {"invert_pitch": False, "invert_roll": False})

        # Deadband y soft-start
        self.deadband = float(self.p.get("deadband_deg", 0.0))
        self.softstart = SoftStart(self.p.get("soft_start_s", 0.0))

        # Límites (ej. límite de comando a tobillos)
        self.limits = self.p.get("limits", {})

        # Filtros derivativos por eje (pitch/roll)
        self._lp_d_pitch = None
        self._lp_d_roll = None

        # LPF de salida: se configura con fc_cmd_hz = 1/(2π τ)
        self._lp_cmd_pitch = None
        self._lp_cmd_roll  = None
        self._cmd_tau_s = float(self.p.get("cmd_lpf_tau_s", 0.08))
        self._cmd_fc_hz = 1.0 / max(1e-6, 2.0 * math.pi * self._cmd_tau_s)

        # Quiet zone para evitar “baile” cerca del equilibrio
        self._quiet_zone_deg = float(self.p.get("quiet_zone_deg", 0.6))

        # Integradores (en deg*s)
        self.i_pitch = 0.0
        self.i_roll = 0.0
        self.i_lim_deg = float(self.p.get("i_limit_deg", 8.0))

        # Cache últimos errores (para D discreto)
        self._prev_p = 0.0
        self._prev_r = 0.0

        # Signos por joint: acepta por-joint (j04_...) o genéricos
        js = self.p.get("joint_signs", {}) or {}
        self.s_ap_l = float(js.get("ankle_pitch_l", js.get("ankle_pitch", 1.0)))
        self.s_hp_l = float(js.get("hip_pitch_l",   js.get("hip_pitch",   1.0)))
        self.s_ar_l = float(js.get("ankle_roll_l",  js.get("ankle_roll",  1.0)))
        self.s_hr_l = float(js.get("hip_roll_l",    js.get("hip_roll",    1.0)))

        self.s_ap_r = float(js.get("ankle_pitch_r", js.get("ankle_pitch", 1.0)))
        self.s_hp_r = float(js.get("hip_pitch_r",   js.get("hip_pitch",   1.0)))
        self.s_ar_r = float(js.get("ankle_roll_r",  js.get("ankle_roll",  1.0)))
        self.s_hr_r = float(js.get("hip_roll_r",    js.get("hip_roll",    1.0)))

    def _ensure_filters(self, dt: float) -> None:
        """
        Asegura que los filtros internos (derivativos y de salida) estén
        inicializados y con la frecuencia correcta para el dt actual.
        """
        # Derivativos
        fc_p = float(self.p.get("fc_d_pitch_hz", 12.0))
        fc_r = float(self.p.get("fc_d_roll_hz", 10.0))
        if self._lp_d_pitch is None:
            self._lp_d_pitch = Lowpass(fc_p, dt)
        else:
            self._lp_d_pitch.set(fc_p, dt)
        if self._lp_d_roll is None:
            self._lp_d_roll = Lowpass(fc_r, dt)
        else:
            self._lp_d_roll.set(fc_r, dt)

        # LPF de salida (en Hz, derivado de τ)
        self._cmd_fc_hz = 1.0 / max(1e-6, 2.0 * math.pi * self._cmd_tau_s)
        if self._lp_cmd_pitch is None:
            self._lp_cmd_pitch = Lowpass(self._cmd_fc_hz, dt)
            self._lp_cmd_roll  = Lowpass(self._cmd_fc_hz, dt)
        else:
            self._lp_cmd_pitch.set(self._cmd_fc_hz, dt)
            self._lp_cmd_roll.set(self._cmd_fc_hz, dt)

    def reset(self) -> None:
        """
        Resetea el estado interno del controlador de balance:
        - Integradores
        - Error previo para derivada
        - Soft-start
        """
        self.i_pitch = 0.0
        self.i_roll = 0.0
        self._prev_p = 0.0
        self._prev_r = 0.0
        self.softstart.reset()

    def step(self, dt: float, imu_pitch_rad: float, imu_roll_rad: float) -> dict:
        """
        Ejecuta un paso de control de balance.

        Parámetros:
        - dt:            paso de tiempo [s].
        - imu_pitch_rad: pitch medido por IMU [rad].
        - imu_roll_rad:  roll medido por IMU [rad].

        Devuelve:
        - dict {joint_name: delta_pos_rad} que se suma a la neutral_pose.
        """
        self._ensure_filters(dt)
        g_soft = self.softstart.gain(dt)

        # Error en grados (más cómodo para deadband y Ki)
        err_p = math.degrees(imu_pitch_rad)
        err_r = math.degrees(imu_roll_rad)

        if self.signs.get("invert_pitch", False):
            err_p = -err_p
        if self.signs.get("invert_roll", False):
            err_r = -err_r

        # Deadband
        e_p_db = deadband_deg(err_p, self.deadband)
        e_r_db = deadband_deg(err_r, self.deadband)

        # Quiet zone cerca del deadband: congela D y baja Kp
        in_quiet_p = abs(err_p) < (self.deadband + self._quiet_zone_deg)
        in_quiet_r = abs(err_r) < (self.deadband + self._quiet_zone_deg)

        # Derivada discreta (deg/s) + LPF
        d_p_raw = (e_p_db - self._prev_p) / max(dt, 1e-6)
        d_r_raw = (e_r_db - self._prev_r) / max(dt, 1e-6)
        d_p = 0.0 if in_quiet_p else self._lp_d_pitch.step(d_p_raw)
        d_r = 0.0 if in_quiet_r else self._lp_d_roll.step(d_r_raw)
        self._prev_p = e_p_db
        self._prev_r = e_r_db

        # Integración (si habilitada) en deg*s
        if self.enable_I:
            self.i_pitch = max(-self.i_lim_deg, min(self.i_lim_deg, self.i_pitch + e_p_db * dt))
            self.i_roll  = max(-self.i_lim_deg, min(self.i_lim_deg, self.i_roll  + e_r_db * dt))
        else:
            self.i_pitch = 0.0
            self.i_roll = 0.0

        # PID (PD + I opcional), en grados
        kp_p = float(self.gains.get("pitch", {}).get("kp", 0.0))
        kd_p = float(self.gains.get("pitch", {}).get("kd", 0.0))
        kp_r = float(self.gains.get("roll",  {}).get("kp", 0.0))
        kd_r = float(self.gains.get("roll",  {}).get("kd", 0.0))
        ki_p = float(self.ki.get("pitch", 0.0)) if self.enable_I else 0.0
        ki_r = float(self.ki.get("roll",  0.0)) if self.enable_I else 0.0

        # Atenúa Kp en quiet zone para evitar “baile”
        if in_quiet_p:
            kp_p *= 0.65
        if in_quiet_r:
            kp_r *= 0.65

        u_p_deg = kp_p * e_p_db + kd_p * d_p + ki_p * self.i_pitch
        u_r_deg = kp_r * e_r_db + kd_r * d_r + ki_r * self.i_roll

        # A rad y pasa por LPF de salida + softstart
        u_p = self._lp_cmd_pitch.step(math.radians(u_p_deg)) * g_soft
        u_r = self._lp_cmd_roll.step(math.radians(u_r_deg))  * g_soft

        # Mezcla a articulaciones
        mix_pa = float(self.mix.get("pitch_to_ankle", 0.22))   # default usado en tus params
        mix_ra = float(self.mix.get("roll_to_ankle",  0.70))

        u_ap = u_p * mix_pa
        u_hp = u_p * (1.0 - mix_pa)
        u_ar = u_r * mix_ra
        u_hr = u_r * (1.0 - mix_ra)

        # Límite de comando a tobillos (para no romper gait)
        ankle_cmd_lim = float(self.limits.get("ankle_cmd", 0.03))
        u_ap = max(-ankle_cmd_lim, min(ankle_cmd_lim, u_ap))

        # Aplica signos por joint (por-joint si viene en params)
        deltas = {
            # Left
            "j04_ankle_pitch_l": self.s_ap_l * u_ap,
            "j00_hip_pitch_l":   self.s_hp_l * u_hp,
            "j05_ankle_roll_l":  self.s_ar_l * u_ar,
            "j01_hip_roll_l":    self.s_hr_l * u_hr,
            # Right
            "j10_ankle_pitch_r": self.s_ap_r * u_ap,
            "j06_hip_pitch_r":   self.s_hp_r * u_hp,
            "j11_ankle_roll_r":  self.s_ar_r * u_ar,
            "j07_hip_roll_r":    self.s_hr_r * u_hr,
        }

        return deltas
