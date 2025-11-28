# src/mcvilloid/controllers/mcvilloid_controller/balance/filters.py
import math


class Lowpass:
    """
    Filtro pasabajas de primer orden (discreto).

    Modelo continuo:
        y'(t) = (x(t) - y(t)) / RC

    Discretizado como:
        y[k] = y[k-1] + α * (x[k] - y[k-1])

    donde:
        RC  = 1 / (2π fc)
        α   = dt / (RC + dt)

    Parámetros:
        fc_hz : frecuencia de corte en Hz.
        dt    : periodo de muestreo en segundos.
        x0    : valor inicial de la salida.
    """
    def __init__(self, fc_hz, dt, x0: float = 0.0):
        self.x = x0
        self.set(fc_hz, dt)

    def set(self, fc_hz, dt) -> None:
        """
        Reconfigura el filtro con nueva fc y/o nuevo dt.

        - fc_hz se satura a un mínimo para evitar división por cero.
        - dt se satura a un mínimo para evitar problemas numéricos.
        """
        fc_hz = max(0.001, float(fc_hz))
        dt = max(1e-6, float(dt))
        rc = 1.0 / (2.0 * math.pi * fc_hz)
        self.alpha = dt / (rc + dt)

    def step(self, x_new: float) -> float:
        """
        Aplica un paso del filtro al valor x_new y devuelve la
        nueva salida filtrada.
        """
        self.x = self.x + self.alpha * (x_new - self.x)
        return self.x


def deadband_deg(x_deg: float, band_deg: float) -> float:
    """
    Aplica una "deadband" (zona muerta) simétrica en grados.

    - Si |x_deg| <= band_deg → devuelve 0.
    - Si |x_deg| > band_deg  → devuelve un valor lineal
      desplazado, manteniendo signo:

        y = sign(x) * (|x| - band_deg)

    Útil para evitar responder a errores muy pequeños (ruido).
    """
    if abs(x_deg) <= band_deg:
        return 0.0
    # lineal fuera del umbral
    return math.copysign(abs(x_deg) - band_deg, x_deg)


class SoftStart:
    """
    Rampa de ganancia suave de 0 → 1 en T segundos.

    - Internamente lleva un tiempo acumulado t en [0, T].
    - gain(dt) aumenta t con dt y devuelve g(t) suavizado
      con un polinomio tipo "ease-in-out":

        g_lin = t / T
        g     = g_lin² * (3 - 2 g_lin)

    Si T <= 0, la ganancia es siempre 1.0 (sin rampa).
    """
    def __init__(self, T_s: float):
        self.T = max(0.0, float(T_s))
        self.t = 0.0

    def reset(self) -> None:
        """Resetea el temporizador interno a 0 (ganancia vuelve a 0)."""
        self.t = 0.0

    def gain(self, dt: float) -> float:
        """
        Avanza el tiempo interno con dt y devuelve la ganancia actual.

        Parámetros:
            dt : incremento de tiempo en segundos (si es negativo, se ignora).

        Retorna:
            g ∈ [0, 1] ganancia suavizada.
        """
        if self.T <= 0.0:
            return 1.0

        self.t = min(self.T, self.t + max(0.0, float(dt)))
        g = self.t / self.T
        # suavizado (ease-in-out)
        return g * g * (3 - 2 * g)
