# control_utils.py
from __future__ import annotations

import math
from typing import Iterable, Tuple

def clamp(x: float, lo: float, hi: float) -> float:
    """Satura x al rango [lo, hi]."""
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, x))

def radians_clamp(x: float, lim: float) -> float:
    """Satura x al rango [-|lim|, +|lim|] en radianes."""
    lim = abs(lim)
    return clamp(x, -lim, +lim)

def clamp_dict(d: dict, limits: dict) -> dict:
    """Devuelve un dict nuevo con cada clave saturada por sus límites si existen.
    limits: { 'joint_name': (lo, hi), ... }
    """
    out = {}
    for k, v in d.items():
        if k in limits:
            lo, hi = limits[k]
            out[k] = clamp(v, lo, hi)
        else:
            out[k] = v
    return out

class LowPass:
    def __init__(self, cutoff_hz, dt):
        rc = 1.0 / (2.0 * math.pi * cutoff_hz)
        self.alpha = dt / (rc + dt)
        self.y = 0.0
        self.init = False
    def step(self, x):
        if not self.init:
            self.y = x; self.init = True
        else:
            self.y = self.y + self.alpha * (x - self.y)
        return self.y

class ComplementaryIMU:
    """Funde acelerómetro + giroscopio para roll/pitch estables."""
    def __init__(self, alpha=0.985, dt=0.032):
        self.alpha = alpha
        self.dt = dt
        self.roll = 0.0
        self.pitch = 0.0
        self.init = False
    def step(self, ax, ay, az, gx, gy):
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        if not self.init:
            self.roll, self.pitch = roll_acc, pitch_acc
            self.init = True
        roll_gyro  = self.roll  + gx * self.dt
        pitch_gyro = self.pitch + gy * self.dt
        self.roll  = self.alpha * roll_gyro  + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        return self.roll, self.pitch

class SmoothPID:
    """PID con D en la medida, anti-windup y limitador de pendiente (slew-rate)."""
    def __init__(self, kp, ki, kd, dt,
                 u_min=-float('inf'), u_max=float('inf'),
                 du_max_per_s=float('inf'),
                 deadband=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.u_min, self.u_max = u_min, u_max
        self.du_max = du_max_per_s * dt
        self.deadband = deadband
        self.integral = 0.0
        self.prev_meas = None
        self.prev_u = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_meas = None
        self.prev_u = 0.0

    def step(self, ref, meas):
        e = ref - meas
        if abs(e) < self.deadband:
            e = 0.0

        d_meas = 0.0
        if self.prev_meas is not None:
            d_meas = (meas - self.prev_meas) / self.dt
        self.prev_meas = meas

        p = self.kp * e
        i = self.integral + self.ki * e * self.dt
        u_unclamped = p + i - self.kd * d_meas

        u = max(self.u_min, min(self.u_max, u_unclamped))
        if u == u_unclamped:  # anti-windup
            self.integral = i

        du = u - self.prev_u
        if abs(du) > self.du_max:
            u = self.prev_u + (self.du_max if du > 0 else -self.du_max)

        self.prev_u = u
        return u
