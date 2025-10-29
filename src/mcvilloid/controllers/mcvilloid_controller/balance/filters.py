# src/mcvilloid/controllers/mcvilloid_controller/balance/filters.py
import math

class Lowpass:
    def __init__(self, fc_hz, dt, x0=0.0):
        self.x = x0
        self.set(fc_hz, dt)

    def set(self, fc_hz, dt):
        fc_hz = max(0.001, float(fc_hz))
        dt = max(1e-6, float(dt))
        rc = 1.0 / (2.0 * math.pi * fc_hz)
        self.alpha = dt / (rc + dt)

    def step(self, x_new):
        self.x = self.x + self.alpha * (x_new - self.x)
        return self.x

def deadband_deg(x_deg: float, band_deg: float) -> float:
    if abs(x_deg) <= band_deg:
        return 0.0
    # lineal fuera del umbral
    return math.copysign(abs(x_deg) - band_deg, x_deg)

class SoftStart:
    """Rampa de ganancia 0..1 en T segundos."""
    def __init__(self, T_s: float):
        self.T = max(0.0, float(T_s))
        self.t = 0.0
    def reset(self):
        self.t = 0.0
    def gain(self, dt: float) -> float:
        if self.T <= 0.0:
            return 1.0
        self.t = min(self.T, self.t + max(0.0, float(dt)))
        g = self.t / self.T
        # suavizado (ease-in-out)
        return g * g * (3 - 2 * g)
