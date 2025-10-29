import math

def cycloid_step(x0, x1, z0, clearance, phase):
    """
    Trayectoria simple del pie en X (avance) y Z (altura) con campana suave.
    phase en [0,1].
    """
    phase = max(0.0, min(1.0, phase))
    x = x0 + (x1 - x0) * phase
    # z: campana (1 - cos(pi*phase)) / 2
    z = z0 + clearance * (1.0 - math.cos(math.pi * phase)) * 0.5
    return x, z

def ease_in_out(t):
    """Suavizado básico para transiciones (0..1)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)
