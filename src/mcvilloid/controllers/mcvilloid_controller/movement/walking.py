# src/mcvilloid/movement/walking.py
# -*- coding: utf-8 -*-
"""
Walker (FSM) para PM01
----------------------
- Constructor compatible con tu controller: Walker(params, limits, logger=None)
- Métodos usados por tu controller:
    - toggle(on: bool)
    - set_global_gain(g: float)
    - step(dt: float, imu: dict, sensors: dict) -> Dict[str, float]
- Atributos que lees en logs:
    - .on (bool)
    - .dir (float)   # +1 forward, -1 backward
    - .phase (float) # 0..1
    - ._global_gain (float)  # (interno, pero lo usas en logs)

Características:
- Rodilla más alta en swing (knee_swing_max_rad)
- Boost de cadera en swing (hip_swing_boost_rad)
- Dorsiflexión en mitad de swing (dorsi_mid_rad, ventana [a,b])
- Trayectoria de pie parabólica (clearance_m) — usada para referencia (IK opcional)
- Limitadores de velocidad (slew) anti-“patada”
- Gate de estabilidad por IMU (reduce amplitudes si tilt)
- FSM interna con cuatro fases: STANCE_L → SWING_R → STANCE_R → SWING_L
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Callable


# ====== Utilidades ======

def clamp(x: float, lo: float, hi: float) -> float:
    return hi if x > hi else lo if x < lo else x

def bell_window(x: float, a: float, b: float) -> float:
    """Ventana suave con pico en mitad del intervalo [a,b]."""
    if x < a or x > b:
        return 0.0
    t = (x - a) / (b - a)
    return 4.0 * t * (1.0 - t)  # máximo 1 en el centro

def foot_height(phi: float, hmax: float) -> float:
    """Altura de pie parabólica: z = 4*h*phi*(1-phi); phi∈[0,1]."""
    return 4.0 * hmax * phi * (1.0 - phi)

def slew(prev: float, target: float, rate: float, dt: float) -> float:
    """Limitador de derivada |dθ/dt| <= rate."""
    up = prev + rate * dt
    dn = prev - rate * dt
    if target > up:  return up
    if target < dn:  return dn
    return target


# ====== Modelos de parámetros ======

@dataclass
class GaitParams:
    # Tiempos
    freq_hz: float = 0.30          # frecuencia base (alternativa a swing/stance explícitos)
    swing_time_s: float = 0.45
    stance_time_s: float = 0.55

    # Amplitudes base
    hip_swing_boost_rad: float = 0.06   # flexión extra de cadera en swing
    knee_swing_max_rad: float = 0.24    # pico de rodilla en swing
    ankle_base_rad: float = 0.00        # base ankle en swing (además de la dorsi media)

    # Clearance / Dorsiflexión
    foot_clearance_m: float = 0.03
    dorsi_mid_rad: float = 0.04
    dorsi_window_a: float = 0.30
    dorsi_window_b: float = 0.70

    # Limitadores de velocidad
    knee_vel_max: float = 2.3
    ankle_vel_max: float = 2.0
    hip_vel_max: float = 2.0

    # Guardas IMU (además de los del controller, aquí es un refuerzo)
    imu_pitch_deg_gate: float = 5.0
    imu_gate_scale: float = 0.7

    # Otros
    enabled_on_start: bool = False
    auto_on: bool = False


@dataclass
class JointMap:
    # Por defecto usamos los nombres que aparecen en tus logs y controller
    hip_l: str = "j00_hip_pitch_l"
    knee_l: str = "j03_knee_pitch_l"
    ankle_l: str = "j04_ankle_pitch_l"
    hip_r: str = "j06_hip_pitch_r"
    knee_r: str = "j09_knee_pitch_r"
    ankle_r: str = "j10_ankle_pitch_r"


@dataclass
class SlewState:
    prev: Dict[str, float] = field(default_factory=dict)


class Phase(Enum):
    STANCE_L = 0
    SWING_R  = 1
    STANCE_R = 2
    SWING_L  = 3


# ====== Walker ======

class Walker:
    def __init__(self, params: dict, limits: dict, logger: Optional[Callable[..., None]] = None):
        self._log = (logger or (lambda *a, **k: None))

        # --- Parseo de params.json (sección "gait") ---
        G = params.get("gait", {}) or {}

        # Tiempos
        freq_hz = float(G.get("freq_hz", 0.30))
        swing_time_s  = float(G.get("swing_time_s", 0.45))
        stance_time_s = float(G.get("stance_time_s", 0.55))
        # Si el usuario sólo dio freq_hz, reparte tiempos básicos
        if "swing_time_s" not in G and "stance_time_s" not in G:
            # regla simple: 45/55
            T = 1.0 / max(1e-6, freq_hz)
            swing_time_s  = 0.45 * T
            stance_time_s = 0.55 * T

        step_amp = G.get("step_amp", {})
        hip_boost = float(step_amp.get("hip_pitch", 0.06))
        knee_max  = float(step_amp.get("knee_pitch", 0.24))
        ankle_base = float(step_amp.get("ankle_pitch", 0.00))

        self.g = GaitParams(
            freq_hz=freq_hz,
            swing_time_s=swing_time_s,
            stance_time_s=stance_time_s,
            hip_swing_boost_rad=hip_boost,
            knee_swing_max_rad=knee_max,
            ankle_base_rad=ankle_base,
            foot_clearance_m=float(G.get("clearance_m", 0.03)),
            dorsi_mid_rad=float(G.get("dorsi_mid_rad", 0.04)),
            dorsi_window_a=float(G.get("dorsi_window", [0.30, 0.70])[0]),
            dorsi_window_b=float(G.get("dorsi_window", [0.30, 0.70])[1]),
            knee_vel_max=float(G.get("slew_rad_s", {}).get("knee", 2.3)),
            ankle_vel_max=float(G.get("slew_rad_s", {}).get("ankle", 2.0)),
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

        # Estado
        self.on: bool = bool(self.g.enabled_on_start)
        self._global_gain: float = 1.0  # el controller lo escala con set_global_gain()
        self.dir: float = +1.0          # +1: forward, -1: backward

        self.phase: float = 0.0         # 0..1 dentro de la fase actual
        self._phase_time: float = 0.0
        self._phase_dur: float = self.g.stance_time_s  # inicia con STANCE_L
        self._phase: Phase = Phase.STANCE_L

        # Slew anterior por joint
        self._slew = SlewState(prev={
            self.jm.hip_l: 0.0, self.jm.knee_l: 0.0, self.jm.ankle_l: 0.0,
            self.jm.hip_r: 0.0, self.jm.knee_r: 0.0, self.jm.ankle_r: 0.0,
        })

        self._log("Walker init",
                  f"freq={self.g.freq_hz:.2f}Hz swing={self.g.swing_time_s:.2f}s stance={self.g.stance_time_s:.2f}s",
                  f"knee_max={self.g.knee_swing_max_rad:.2f} hip_boost={self.g.hip_swing_boost_rad:.2f}",
                  f"dorsi={self.g.dorsi_mid_rad:.2f} window=[{self.g.dorsi_window_a:.2f},{self.g.dorsi_window_b:.2f}]",
                  f"clearance={self.g.foot_clearance_m*1000:.0f}mm")

    # ---------- API usada por el controller ----------

    def toggle(self, on: bool):
        """Enciende/Apaga el gait. Cuando está apagado, step() devuelve {}."""
        self.on = bool(on)

    def set_global_gain(self, g: float):
        """El controller te pasa un gain (0..1) según sus tilt/guards."""
        self._global_gain = clamp(float(g), 0.0, 1.0)

    # imu: {"pitch": rad, "roll": rad}
    # sensors: reservado para contacto si lo agregas después
    def step(self, dt: float, imu: Dict[str, float], sensors: Dict) -> Dict[str, float]:
        if not self.on:
            return {}

        # Avance de fase
        self._phase_time += dt
        if self._phase_time >= self._phase_dur:
            self._next_phase()

        # Progresión local 0..1 dentro de la fase
        self.phase = clamp(self._phase_time / max(1e-6, self._phase_dur), 0.0, 1.0)

        # Gate IMU interno (suave, adicional al del controller)
        imu_pitch_deg = float(imu.get("pitch", 0.0)) * (180.0 / math.pi)
        amp_scale_imu = 1.0 if abs(imu_pitch_deg) <= self.g.imu_pitch_deg_gate else self.g.imu_gate_scale

        # Ganancia total
        GAIN = self._global_gain * amp_scale_imu
        if GAIN <= 1e-6:
            return {}

        # Construir consignas
        cmd: Dict[str, float] = {
            self.jm.hip_l: 0.0, self.jm.knee_l: 0.0, self.jm.ankle_l: 0.0,
            self.jm.hip_r: 0.0, self.jm.knee_r: 0.0, self.jm.ankle_r: 0.0,
        }

        # Selecciona pierna en swing según fase
        if self._phase == Phase.SWING_L:
            self._apply_swing_left(cmd, self.phase, GAIN, dt)
        elif self._phase == Phase.SWING_R:
            self._apply_swing_right(cmd, self.phase, GAIN, dt)
        else:
            # STANCE_*: podrías añadir pequeños ajustes si lo deseas
            pass

        return cmd

    # ---------- Internos ----------

    def _next_phase(self):
        # Ciclo: STANCE_L → SWING_R → STANCE_R → SWING_L → ...
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

    def _apply_swing_left(self, cmd: Dict[str, float], phi: float, gain: float, dt: float):
        g = self.g
        # cadera: boost en swing (dirección no invierte el boost; la direcc. afecta avance/retorno del paso)
        hip_target   = gain * g.hip_swing_boost_rad
        # rodilla: senoide 0..π → 0..1..0
        knee_target  = gain * g.knee_swing_max_rad * math.sin(math.pi * phi)
        # tobillo: base + dorsiflexión en mitad de swing
        ankle_target = gain * (g.ankle_base_rad + g.dorsi_mid_rad * bell_window(phi, g.dorsi_window_a, g.dorsi_window_b))

        # (IK opcional) z objetivo del pie
        _z_clear = foot_height(phi, g.foot_clearance_m)  # disponible si luego lo conectas a IK

        # Slew
        cmd[self.jm.hip_l]   = slew(self._slew.prev[self.jm.hip_l],   hip_target,   g.hip_vel_max,   dt)
        cmd[self.jm.knee_l]  = slew(self._slew.prev[self.jm.knee_l],  knee_target,  g.knee_vel_max,  dt)
        cmd[self.jm.ankle_l] = slew(self._slew.prev[self.jm.ankle_l], ankle_target, g.ankle_vel_max, dt)

        # Guardar previos
        self._slew.prev[self.jm.hip_l]   = cmd[self.jm.hip_l]
        self._slew.prev[self.jm.knee_l]  = cmd[self.jm.knee_l]
        self._slew.prev[self.jm.ankle_l] = cmd[self.jm.ankle_l]

        # Pierna contraria (stance) podría llevar un micro-contrapeso si se necesita
        # (lo dejamos en 0 para que tu BalanceController lleve la batuta)

    def _apply_swing_right(self, cmd: Dict[str, float], phi: float, gain: float, dt: float):
        g = self.g
        hip_target   = gain * g.hip_swing_boost_rad
        knee_target  = gain * g.knee_swing_max_rad * math.sin(math.pi * phi)
        ankle_target = gain * (g.ankle_base_rad + g.dorsi_mid_rad * bell_window(phi, g.dorsi_window_a, g.dorsi_window_b))

        _z_clear = foot_height(phi, g.foot_clearance_m)

        cmd[self.jm.hip_r]   = slew(self._slew.prev[self.jm.hip_r],   hip_target,   g.hip_vel_max,   dt)
        cmd[self.jm.knee_r]  = slew(self._slew.prev[self.jm.knee_r],  knee_target,  g.knee_vel_max,  dt)
        cmd[self.jm.ankle_r] = slew(self._slew.prev[self.jm.ankle_r], ankle_target, g.ankle_vel_max, dt)

        self._slew.prev[self.jm.hip_r]   = cmd[self.jm.hip_r]
        self._slew.prev[self.jm.knee_r]  = cmd[self.jm.knee_r]
        self._slew.prev[self.jm.ankle_r] = cmd[self.jm.ankle_r]
