# src/mcvilloid/movement/walking.py
# -*- coding: utf-8 -*-
"""
Walker (FSM) para el gait cíclico del humanoide Mcvilloid
--------------------------------------------------------

API esperada por el controller:

    walker = Walker(params, limits, logger=None)

Métodos usados por el controller:

    - walker.toggle(on: bool)
    - walker.set_global_gain(g: float)
    - walker.set_stride_base(s: float)
    - walker.step(dt: float, imu: dict, sensors: dict) -> Dict[str, float]

Atributos que se leen en logs:

    - walker.on (bool)
    - walker.dir (float)    # +1 forward, -1 backward
    - walker.phase (float)  # 0..1 dentro de la fase actual
    - walker._global_gain   # ganancia global interna (para debug/log)

Características principales:

    - FSM interna de 4 fases: STANCE_L → SWING_R → STANCE_R → SWING_L.
    - Swing con rodilla alta y dorsiflexión media (ventana configurable).
    - Bias de cadera hacia adelante para adelantar el COM.
    - Toe-off en la pierna de apoyo con edge-trigger + refractario.
    - Gate por IMU (pitch) que recorta amplitudes con histéresis.
    - Slew-rate por articulación para evitar “patadas” (limitador de derivada).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Callable


# ====== Utilidades ======


def clamp(x: float, lo: float, hi: float) -> float:
    """Satura x al intervalo [lo, hi]."""
    return hi if x > hi else lo if x < lo else x


def bell_window(x: float, a: float, b: float) -> float:
    """
    Ventana suave en [a, b] con máximo 1 en el centro.

    Se usa para aplicar dorsiflexión sólo en una subventana del swing.
    """
    if x < a or x > b:
        return 0.0
    t = (x - a) / (b - a)
    return 4.0 * t * (1.0 - t)  # máximo 1 en el centro


def slew(prev: float, target: float, rate: float, dt: float) -> float:
    """
    Limitador de derivada: garantiza que |θ_dot| <= rate.

    Devuelve un nuevo valor intermedio que se mueve desde prev hacia target
    con una máxima variación de ± rate * dt.
    """
    up = prev + rate * dt
    dn = prev - rate * dt
    if target > up:
        return up
    if target < dn:
        return dn
    return target


# Ganancias sencillas para la pierna en apoyo (stance) y toe-off
STANCE_HIP_COEFF = 0.25      # cadera en apoyo ≈ +0.25 * hip_swing (opuesto al swing)
TOEOFF_ANKLE_RAD = 0.050     # empuje plantar en apoyo (rad)
TOEOFF_WIN_L = (0.85, 0.98)  # ventana de toe-off cuando la otra pierna está en SWING_L
TOEOFF_WIN_R = (0.85, 0.98)  # ventana de toe-off cuando la otra pierna está en SWING_R

# Bias de cadera hacia adelante (suaviza inicio de paso)
HIP_WALK_FWD_BIAS_RAD = 0.05

# Gate de IMU con histéresis (en grados)
IMU_GATE_ON_DEG = 8.0    # entra a modo recortado si |pitch| > 8°
IMU_GATE_OFF_DEG = 6.0   # sale cuando |pitch| < 6°


def bell01(x: float, a: float, b: float) -> float:
    """
    Ventana 0..1 en [a, b] con pico en el centro.
    Similar a bell_window, pero normalizada explícitamente a 0..1.
    """
    if x < a or x > b:
        return 0.0
    t = (x - a) / max(1e-6, (b - a))
    return 4.0 * t * (1.0 - t)


def in_window(x: float, a: float, b: float) -> float:
    """
    Devuelve 0 fuera de [a, b] y una rampa 0..1 dentro del intervalo.
    Útil para edge-triggers (detección de entrada a ventana).
    """
    return 0.0 if (x < a or x > b) else (x - a) / max(1e-6, (b - a))


# ====== Modelos de parámetros ======


@dataclass
class GaitParams:
    """
    Parámetros cinemáticos y de timing del gait cíclico.
    """
    # Tiempos
    freq_hz: float = 0.30          # frecuencia base
    swing_time_s: float = 0.45
    stance_time_s: float = 0.55

    # Amplitudes base
    hip_swing_boost_rad: float = 0.06   # flexión extra de cadera en swing
    knee_swing_max_rad: float = 0.24    # pico de rodilla en swing
    ankle_base_rad: float = 0.00        # offset base de tobillo en swing

    # Clearance / Dorsiflexión
    foot_clearance_m: float = 0.03      # parámetro de referencia (se usa en logs)
    dorsi_mid_rad: float = 0.04
    dorsi_window_a: float = 0.30
    dorsi_window_b: float = 0.70

    # Limitadores de velocidad (rad/s)
    knee_vel_max: float = 2.3
    ankle_vel_max: float = 2.0
    hip_vel_max: float = 2.0

    # Guardas IMU (refuerzo adicional sobre el controller)
    imu_pitch_deg_gate: float = 5.0
    imu_gate_scale: float = 0.7

    # Otros flags
    enabled_on_start: bool = False
    auto_on: bool = False


@dataclass
class JointMap:
    """
    Mapa de nombres lógicos de articulaciones para el gait.
    Por defecto coincide con los nombres usados en los logs y el controller.
    """
    hip_l: str = "j00_hip_pitch_l"
    knee_l: str = "j03_knee_pitch_l"
    ankle_l: str = "j04_ankle_pitch_l"
    hip_r: str = "j06_hip_pitch_r"
    knee_r: str = "j09_knee_pitch_r"
    ankle_r: str = "j10_ankle_pitch_r"


@dataclass
class SlewState:
    """
    Estado para el slew-rate por articulación.
    Guarda el último comando aplicado.
    """
    prev: Dict[str, float] = field(default_factory=dict)


class Phase(Enum):
    """Fases del ciclo de marcha."""
    STANCE_L = 0
    SWING_R = 1
    STANCE_R = 2
    SWING_L = 3


# ====== Walker ======


class Walker:
    """
    Generador de gait cíclico para las articulaciones de pitch de cadera,
    rodilla y tobillo de ambas piernas.

    No aplica offsets absolutos, sino deltas respecto a una pose neutra que
    el controller suma luego a `base_pose`.
    """

    def __init__(self, params: dict, limits: dict, logger: Optional[Callable[..., None]] = None):
        # logger genérico: se espera algo tipo RateLogger.info/debug
        self._log = (logger or (lambda *a, **k: None))

        # --- Parseo de params.json (sección "gait") ---
        G = params.get("gait", {}) or {}
        self.Gfull = dict(G)  # copia segura de params["gait"]
        if not isinstance(self.Gfull.get("limits"), dict):
            self.Gfull["limits"] = {}

        # Ganancia extra de altura en swing (rodilla/tobillo)
        # Ajustable desde params.json:
        #   "knee_swing_gain": 2.0, "ankle_swing_gain": 1.5
        self._knee_gain = float(G.get("knee_swing_gain", 2.0))
        self._ankle_gain = float(G.get("ankle_swing_gain", 1.5))

        # Tiempos
        freq_hz = float(G.get("freq_hz", 0.30))
        swing_time_s = float(G.get("swing_time_s", 0.45))
        stance_time_s = float(G.get("stance_time_s", 0.55))

        # Si el usuario sólo dio freq_hz, repartimos tiempos básicos 45/55
        if "swing_time_s" not in G and "stance_time_s" not in G:
            T = 1.0 / max(1e-6, freq_hz)
            swing_time_s = 0.45 * T
            stance_time_s = 0.55 * T

        step_amp = G.get("step_amp", {})
        hip_boost = float(step_amp.get("hip_pitch", 0.06))
        knee_max_unused = float(step_amp.get("knee_pitch", 0.30))  # sólo referencia; el valor real se fija abajo
        ankle_base = float(step_amp.get("ankle_pitch", 0.00))

        self.g = GaitParams(
            freq_hz=freq_hz,
            swing_time_s=swing_time_s,
            stance_time_s=stance_time_s,
            hip_swing_boost_rad=hip_boost,
            knee_swing_max_rad=float(step_amp.get("knee_pitch", 0.35)),
            knee_vel_max=float(G.get("slew_rad_s", {}).get("knee", 1.8)),  # antes 2.3
            ankle_base_rad=ankle_base,
            foot_clearance_m=float(G.get("clearance_m", 0.03)),
            dorsi_mid_rad=float(G.get("dorsi_mid_rad", 0.06)),
            dorsi_window_a=float(G.get("dorsi_window", [0.30, 0.70])[0]),
            dorsi_window_b=float(G.get("dorsi_window", [0.30, 0.70])[1]),
            ankle_vel_max=float(G.get("slew_rad_s", {}).get("ankle", 2.3)),
            hip_vel_max=float(G.get("slew_rad_s", {}).get("hip", 2.0)),
            imu_pitch_deg_gate=float(G.get("imu_pitch_deg_gate", 5.0)),
            imu_gate_scale=float(G.get("imu_gate_scale", 0.7)),
            enabled_on_start=bool(G.get("enabled_on_start", False)),
            auto_on=bool(G.get("auto_on", False)),
        )

        # Joint map (permitimos override desde params.json -> "joint_map")
        JM = params.get("joint_map", {}) or {}
        self.jm = JointMap(
            hip_l=JM.get("hip_l", JointMap.hip_l),
            knee_l=JM.get("knee_l", JointMap.knee_l),
            ankle_l=JM.get("ankle_l", JointMap.ankle_l),
            hip_r=JM.get("hip_r", JointMap.hip_r),
            knee_r=JM.get("knee_r", JointMap.knee_r),
            ankle_r=JM.get("ankle_r", JointMap.ankle_r),
        )

        # Estado global de encendido
        self.on: bool = bool(self.g.enabled_on_start)

        # Ganancia global (para “softener” del controller)
        self._global_gain: float = 1.0          # gain actual (suavizado)
        self._global_gain_target: float = 1.0   # gain objetivo
        self._gain_slew_rad_s: float = 1.5      # rapidez de cambio del gain

        # Dirección del gait: +1 forward, -1 backward
        self.dir: float = +1.0

        # Fase local dentro de la fase actual (0..1)
        self.phase: float = 0.0

        # --- Escala de zancada (stride) ---
        Gfull = params.get("gait", {}) or {}
        self.stride_base: float = float(Gfull.get("stride_base", 1.20))
        self._stride_max_param: float = float(Gfull.get("stride_max", 1.35))
        self._stride_min_param: float = 0.60

        # FSM interna
        self._phase_time: float = 0.0
        self._phase_dur: float = self.g.stance_time_s  # inicia con STANCE_L
        self._phase: Phase = Phase.STANCE_L

        # Slew anterior por joint
        self._slew = SlewState(prev={
            self.jm.hip_l: 0.0,
            self.jm.knee_l: 0.0,
            self.jm.ankle_l: 0.0,
            self.jm.hip_r: 0.0,
            self.jm.knee_r: 0.0,
            self.jm.ankle_r: 0.0,
        })

        # --- Signos por joint (desde params.json -> "joint_signs") ---
        JS = params.get("joint_signs") or {}
        self.signs = {
            self.jm.hip_l: float(JS.get("hip_pitch_l", 1.0)),
            self.jm.knee_l: float(JS.get("knee_pitch_l", 1.0)),
            self.jm.ankle_l: float(JS.get("ankle_pitch_l", 1.0)),
            self.jm.hip_r: float(JS.get("hip_pitch_r", 1.0)),
            self.jm.knee_r: float(JS.get("knee_pitch_r", 1.0)),
            self.jm.ankle_r: float(JS.get("ankle_pitch_r", 1.0)),
        }
        self._log("Walker signs/dir", f"dir0={self.dir} signs={self.signs}")

        self._log(
            "Walker init",
            f"freq={self.g.freq_hz:.2f}Hz swing={self.g.swing_time_s:.2f}s stance={self.g.stance_time_s:.2f}s",
            f"knee_max={self.g.knee_swing_max_rad:.2f} hip_boost={self.g.hip_swing_boost_rad:.2f}",
            f"dorsi={self.g.dorsi_mid_rad:.2f} window=[{self.g.dorsi_window_a:.2f},{self.g.dorsi_window_b:.2f}]",
            f"clearance={self.g.foot_clearance_m * 1000:.0f}mm",
        )

        # Estados para toe-off edge-trigger
        self._toe_w_prev_L = 0.0
        self._toe_w_prev_R = 0.0

        # Refractario para toe-off (evita doble impulso por ciclo)
        self._toe_refrac_s = 0.30
        self._toe_timer = 0.0

        # Umbrales auxiliares
        self._knee_late_swing_rad = 0.22   # dispara toe-off si rodilla swing ya va alta
        self._max_roll_deg_for_toe = 10.0  # no toe-off si hay roll fuerte

        # Estado del gate de IMU (escala de amplitud suavizada)
        self._imu_amp_scale = 1.0

        # IMU interna para toe-off / gates (se actualiza en step)
        self._imu_pitch = 0.0
        self._imu_roll = 0.0

    # ---------- API usada por el controller ----------

    def toggle(self, on: bool) -> None:
        """
        Enciende/Apaga el gait.
        Cuando está apagado, step() devuelve {} y se resetea la ganancia global.
        """
        self.on = bool(on)
        if not self.on:
            # Matar cualquier resto de amplitud
            self._global_gain_target = 0.0
            self._global_gain = 0.0

    def set_global_gain(self, g: float) -> None:
        """
        El controller pasa un gain (0..1) según sus tilt/guards.
        Aquí sólo se guarda el target; el slewing se hace en step().
        """
        self._global_gain_target = clamp(float(g), 0.0, 1.0)

    def set_stride_base(self, s: float) -> None:
        """
        Ajusta el multiplicador de zancada (stride) y loguea el nuevo valor.
        """
        self.stride_base = clamp(float(s), self._stride_min_param, self._stride_max_param)
        self._log("gait", f"[STRIDE] base set -> {self.stride_base:.2f}")

    def step(self, dt: float, imu: Dict[str, float], sensors: Dict) -> Dict[str, float]:
        """
        Avanza el gait dt segundos y devuelve un dict {joint_name: delta_rad}
        para sumar a la pose neutra.

        imu:
            {"pitch": rad, "roll": rad}

        sensors:
            reservado para futuros contactos/sensores de pie.
        """
        if not self.on:
            return {}

        # Actualizar IMU interna
        self._imu_pitch = float(imu.get("pitch", 0.0))
        self._imu_roll = float(imu.get("roll", 0.0))

        # Temporizador de refractario toe-off
        if self._toe_timer > 0.0:
            self._toe_timer = max(0.0, self._toe_timer - dt)

        # Avance de fase global
        self._phase_time += dt
        if self._phase_time >= self._phase_dur:
            self._next_phase()

        # Progresión local 0..1 dentro de la fase
        self.phase = clamp(self._phase_time / max(1e-6, self._phase_dur), 0.0, 1.0)

        # 1) Slew del gain global (evita escalones de amplitud en mitad de paso)
        self._global_gain = slew(self._global_gain, self._global_gain_target, self._gain_slew_rad_s, dt)

        # 2) Gate IMU con histéresis + suavizado
        imu_pitch_deg = self._imu_pitch * (180.0 / math.pi)
        if abs(imu_pitch_deg) > IMU_GATE_ON_DEG:
            target_scale = self.g.imu_gate_scale  # p.ej. 0.7
        elif abs(imu_pitch_deg) < IMU_GATE_OFF_DEG:
            target_scale = 1.0
        else:
            # Mantener el valor actual en la zona de histéresis
            target_scale = self._imu_amp_scale

        # Exponencial suave (no escalones bruscos)
        self._imu_amp_scale = 0.85 * self._imu_amp_scale + 0.15 * target_scale

        # 3) Ganancia total
        GAIN = self._global_gain * self._imu_amp_scale
        if GAIN <= 1e-6:
            # Demasiado pequeño → ignora gait
            return {}

        # Comandos iniciales (en rad, sobre pose neutra)
        cmd: Dict[str, float] = {
            self.jm.hip_l: 0.0,
            self.jm.knee_l: 0.0,
            self.jm.ankle_l: 0.0,
            self.jm.hip_r: 0.0,
            self.jm.knee_r: 0.0,
            self.jm.ankle_r: 0.0,
        }

        # Selecciona pierna en swing según fase
        if self._phase == Phase.SWING_L:
            self._apply_swing_left(cmd, self.phase, GAIN, dt)
        elif self._phase == Phase.SWING_R:
            self._apply_swing_right(cmd, self.phase, GAIN, dt)
        else:
            # STANCE_*: ajustes opcionales ya se aplican en las funciones de swing opuesto
            pass

        # --- Clamps intra-Walker (límites fijos de comando) ---
        # OJO: estos son limites locales, el controller aplica otros clamps globales
        hip_min, hip_max = -0.22, +0.22
        knee_min, knee_max = -0.05, +0.60   # deja subir rodilla hasta ~23°
        ankp_min, ankp_max = -0.20, +0.20   # rango moderado en tobillo pitch

        def _cl(v: float, lo: float, hi: float) -> float:
            return hi if v > hi else lo if v < lo else v

        for j in list(cmd.keys()):
            if j in (self.jm.hip_l, self.jm.hip_r):
                cmd[j] = _cl(cmd[j], hip_min, hip_max)
            elif j in (self.jm.knee_l, self.jm.knee_r):
                cmd[j] = _cl(cmd[j], knee_min, knee_max)
            elif j in (self.jm.ankle_l, self.jm.ankle_r):
                cmd[j] = _cl(cmd[j], ankp_min, ankp_max)

        return cmd

    # ---------- Internos ----------

    def _extra_swing_lift(self, phi: float, amp: float) -> tuple[float, float]:
        """
        Extra flexión de rodilla y dorsiflexión de tobillo en la fase de swing
        para aumentar la altura de paso.

        phi:
            fase local 0..1 de la pierna en swing
        amp:
            factor de amplitud (incluye gain * stride_base)
        """
        # Pico en la mitad del swing, suave
        lift = math.sin(math.pi * phi) ** 2  # 0..1..0
        # Escala relativa: ~0.6 veces la rodilla máxima, 0.8 de dorsi mid
        extra_knee = amp * 0.60 * self.g.knee_swing_max_rad * lift
        extra_ank = amp * 0.80 * self.g.dorsi_mid_rad * lift
        return extra_knee, extra_ank

    def _next_phase(self) -> None:
        """
        Avanza la FSM de fase:
            STANCE_L → SWING_R → STANCE_R → SWING_L → STANCE_L → ...
        y reinicia temporizadores y detectores de toe-off.
        """
        if self._phase == Phase.STANCE_L:
            self._phase = Phase.SWING_R
            self._phase_dur = self.g.swing_time_s
        elif self._phase == Phase.SWING_R:
            self._phase = Phase.STANCE_R
            self._phase_dur = self.g.stance_time_s
        elif self._phase == Phase.STANCE_R:
            self._phase = Phase.SWING_L
            self._phase_dur = self.g.swing_time_s
        else:
            self._phase = Phase.STANCE_L
            self._phase_dur = self.g.stance_time_s

        self._phase_time = 0.0

        # Limpia detectores de borde y evita toe-off inmediato
        self._toe_w_prev_L = 0.0
        self._toe_w_prev_R = 0.0

    def _apply_swing_left(self, cmd: Dict[str, float], phi: float, gain: float, dt: float) -> None:
        """
        Aplica consignas para la fase SWING_L:
        - Pierna izquierda en swing.
        - Pierna derecha en apoyo (stance + toe-off).
        """
        g = self.g

        # Cadera en swing: flexión + bias hacia adelante
        hip_target = gain * g.hip_swing_boost_rad * self.stride_base
        hip_target += gain * HIP_WALK_FWD_BIAS_RAD

        # Escala de swing (altura de paso)
        s = math.sin(math.pi * phi)  # 0..1..0
        if s < 0.0:
            s = 0.0

        amp = gain * self.stride_base

        knee_target = amp * self._knee_gain * g.knee_swing_max_rad * s

        ankle_target = amp * (
            g.ankle_base_rad
            + self._ankle_gain
            * g.dorsi_mid_rad
            * bell_window(phi, g.dorsi_window_a, g.dorsi_window_b)
        )

        extra_knee, extra_ank = self._extra_swing_lift(phi, amp)
        knee_target += extra_knee
        ankle_target += extra_ank

        # Slew (pierna en swing)
        cmd[self.jm.hip_l] = slew(self._slew.prev[self.jm.hip_l], hip_target, g.hip_vel_max, dt)
        cmd[self.jm.knee_l] = slew(self._slew.prev[self.jm.knee_l], knee_target, g.knee_vel_max, dt)
        cmd[self.jm.ankle_l] = slew(self._slew.prev[self.jm.ankle_l], ankle_target, g.ankle_vel_max, dt)

        # --- Pierna de apoyo (derecha): contrapeso de cadera + toe-off edge-trigger ---
        w = bell01(phi, 0.25, 0.85)
        stance_hip = +STANCE_HIP_COEFF * hip_target * w
        cmd[self.jm.hip_r] = slew(self._slew.prev[self.jm.hip_r], stance_hip, g.hip_vel_max, dt)
        self._slew.prev[self.jm.hip_r] = cmd[self.jm.hip_r]

        # Toe-off con edge-trigger + refractario + chequeos extra
        toe_w = in_window(phi, *TOEOFF_WIN_L)  # 0..1 en ventana final
        edge = (toe_w > 0.0) and (self._toe_w_prev_L == 0.0) and (phi > TOEOFF_WIN_L[0])

        roll_deg = abs(self._imu_roll * 180.0 / math.pi)
        knee_swing_L = gain * g.knee_swing_max_rad * math.sin(math.pi * phi)

        can_toe = (
            self._toe_timer == 0.0
            and knee_swing_L >= self._knee_late_swing_rad
            and roll_deg <= self._max_roll_deg_for_toe
        )

        if edge and can_toe and self._imu_pitch > -0.01:
            toe = gain * TOEOFF_ANKLE_RAD * toe_w
            toe_cmd = slew(self._slew.prev[self.jm.ankle_r], -toe, g.ankle_vel_max, dt)
            cmd[self.jm.ankle_r] = toe_cmd
            self._slew.prev[self.jm.ankle_r] = toe_cmd
            self._toe_timer = self._toe_refrac_s  # activa refractario

        self._toe_w_prev_L = toe_w

        # Clamp anti-dorsi en apoyo (derecha) en la mitad del swing opuesto
        if 0.35 <= phi <= 0.65:
            _cur = cmd.get(self.jm.ankle_r, self._slew.prev[self.jm.ankle_r])
            _ank = min(_cur, 0.0)  # no dorsiflexión (>0); permitir plantar/neutral (<=0)
            cmd[self.jm.ankle_r] = _ank
            self._slew.prev[self.jm.ankle_r] = _ank

        # Guardar previos de swing L
        self._slew.prev[self.jm.hip_l] = cmd[self.jm.hip_l]
        self._slew.prev[self.jm.knee_l] = cmd[self.jm.knee_l]
        self._slew.prev[self.jm.ankle_l] = cmd[self.jm.ankle_l]

        # Aplicar signos de hardware
        cmd[self.jm.hip_l] *= self.signs[self.jm.hip_l]
        cmd[self.jm.knee_l] *= self.signs[self.jm.knee_l]
        # Tobillo izquierdo: se deja sin signo si tu hardware ya lo contempla
        cmd[self.jm.hip_r] *= self.signs[self.jm.hip_r]

    def _apply_swing_right(self, cmd: Dict[str, float], phi: float, gain: float, dt: float) -> None:
        """
        Aplica consignas para la fase SWING_R:
        - Pierna derecha en swing.
        - Pierna izquierda en apoyo (stance + toe-off).
        """
        g = self.g

        hip_target = gain * g.hip_swing_boost_rad * self.stride_base
        hip_target += gain * HIP_WALK_FWD_BIAS_RAD

        # Escala de swing (altura de paso)
        s = math.sin(math.pi * phi)  # 0..1..0
        if s < 0.0:
            s = 0.0

        amp = gain * self.stride_base

        knee_target = amp * self._knee_gain * g.knee_swing_max_rad * s

        ankle_target = amp * (
            g.ankle_base_rad
            + self._ankle_gain
            * g.dorsi_mid_rad
            * bell_window(phi, g.dorsi_window_a, g.dorsi_window_b)
        )

        extra_knee, extra_ank = self._extra_swing_lift(phi, amp)
        knee_target += extra_knee
        ankle_target += extra_ank

        # Swing derecho
        cmd[self.jm.hip_r] = slew(self._slew.prev[self.jm.hip_r], hip_target, g.hip_vel_max, dt)
        cmd[self.jm.knee_r] = slew(self._slew.prev[self.jm.knee_r], knee_target, g.knee_vel_max, dt)
        cmd[self.jm.ankle_r] = slew(self._slew.prev[self.jm.ankle_r], ankle_target, g.ankle_vel_max, dt)

        # --- Pierna de apoyo (izquierda): contrapeso de cadera + toe-off edge-trigger ---
        w = bell01(phi, 0.25, 0.85)
        stance_hip = STANCE_HIP_COEFF * hip_target * w
        cmd[self.jm.hip_l] = slew(self._slew.prev[self.jm.hip_l], stance_hip, g.hip_vel_max, dt)
        self._slew.prev[self.jm.hip_l] = cmd[self.jm.hip_l]

        toe_w = in_window(phi, *TOEOFF_WIN_R)
        edge = (toe_w > 0.0) and (self._toe_w_prev_R == 0.0) and (phi > TOEOFF_WIN_R[0])

        roll_deg = abs(self._imu_roll * 180.0 / math.pi)
        knee_swing_R = gain * g.knee_swing_max_rad * math.sin(math.pi * phi)
        can_toe = (
            self._toe_timer == 0.0
            and knee_swing_R >= self._knee_late_swing_rad
            and roll_deg <= self._max_roll_deg_for_toe
        )

        if edge and can_toe and self._imu_pitch > -0.01:
            toe = gain * TOEOFF_ANKLE_RAD * toe_w
            toe_cmd = slew(self._slew.prev[self.jm.ankle_l], -toe, g.ankle_vel_max, dt)
            cmd[self.jm.ankle_l] = toe_cmd
            self._slew.prev[self.jm.ankle_l] = toe_cmd
            self._toe_timer = self._toe_refrac_s

        self._toe_w_prev_R = toe_w

        # Clamp anti-dorsi en apoyo (izquierda) en la mitad del swing opuesto
        if 0.30 <= phi <= 0.70:
            _cur = cmd.get(self.jm.ankle_l, self._slew.prev[self.jm.ankle_l])
            _ank = min(_cur, 0.0)
            cmd[self.jm.ankle_l] = _ank
            self._slew.prev[self.jm.ankle_l] = _ank

        # Guardar previos de swing R
        self._slew.prev[self.jm.hip_r] = cmd[self.jm.hip_r]
        self._slew.prev[self.jm.knee_r] = cmd[self.jm.knee_r]
        self._slew.prev[self.jm.ankle_r] = cmd[self.jm.ankle_r]

        # Aplicar signos de hardware
        cmd[self.jm.hip_r] *= self.signs[self.jm.hip_r]
        cmd[self.jm.knee_r] *= self.signs[self.jm.knee_r]
        # Tobillo derecho igual que el izquierdo (signo opcional)
        cmd[self.jm.hip_l] *= self.signs[self.jm.hip_l]
