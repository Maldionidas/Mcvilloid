# src/mcvilloid/controllers/mcvilloid_controller/rate_logger.py
"""
RateLogger
----------
Logger muy simple con:
- Niveles: debug / info / warn (por canal).
- Rate limit por canal (Hz) para no spamear la consola.
- Filtro por canal configurable en params.json -> "logging": { "channels": { ... } }

Uso típico desde el controller:
    log_cfg = params.get("logging", {})
    LOG = RateLogger(log_cfg, time.monotonic, robot_name=robot_name)

    LOG.info("boot", "mcvilloid_controller: init")
    LOG.debug("imu", f"p={pitch:+.3f} r={roll:+.3f}")
"""

import time
from typing import Callable, Dict, Any


class RateLogger:
    # Mapa de etiqueta de nivel -> prioridad numérica
    LVL = {"debug": 10, "info": 20, "warn": 30}

    def __init__(
        self,
        cfg: Dict[str, Any],
        now_fn: Callable[[], float] | None = None,
        robot_name: str = "???",
    ) -> None:
        """
        Parámetros
        ----------
        cfg : dict
            Sección "logging" de params.json. Claves soportadas:
              - enabled : bool (default True)
              - level   : "debug" | "info" | "warn"
              - rate_hz : frecuencia máxima de log por canal
              - channels: dict { "boot": true, "imu": false, ... }
        now_fn : callable, opcional
            Función de tiempo tipo time.monotonic. Útil para mocks en tests.
        robot_name : str
            Nombre del robot para “taggearlo” en los logs, e.g. "[HU_D04_01]".
        """
        self.enabled: bool = bool(cfg.get("enabled", True))
        self.level: int = self.LVL.get(str(cfg.get("level", "info")).lower(), 20)
        self.rate_hz: float = float(cfg.get("rate_hz", 10.0))
        self.period: float = 1.0 / max(1e-6, self.rate_hz)

        # Permite habilitar/deshabilitar canales específicos:
        # "channels": { "boot": true, "imu": false, "gait": true, ... }
        self.channels: Dict[str, bool] = {
            k: bool(v) for k, v in cfg.get("channels", {}).items()
        }

        self._last: Dict[str, float] = {}          # último tiempo logueado por canal
        self._now: Callable[[], float] = now_fn or time.monotonic
        self.tag: str = f"[{robot_name}]"

    def _ok(self, ch: str, lvl: int) -> bool:
        """
        Devuelve True si:
          - logging global está habilitado,
          - el canal `ch` está habilitado,
          - el nivel `lvl` es >= nivel configurado,
          - ya pasó al menos `period` segundos desde el último log en este canal.
        """
        if not self.enabled:
            return False

        # Filtro por canal: si no está en channels, por defecto lo permitimos (True)
        if not self.channels.get(ch, True):
            return False

        if lvl < self.level:
            return False

        t = self._now()
        if t - self._last.get(ch, -1e9) < self.period:
            return False

        self._last[ch] = t
        return True

    def debug(self, ch: str, msg: str) -> None:
        """Log nivel DEBUG en canal `ch`, sujeto a filtros y rate limit."""
        if self._ok(ch, self.LVL["debug"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)

    def info(self, ch: str, msg: str) -> None:
        """Log nivel INFO en canal `ch`, sujeto a filtros y rate limit."""
        if self._ok(ch, self.LVL["info"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)

    def warn(self, ch: str, msg: str) -> None:
        """Log nivel WARN en canal `ch`, sujeto a filtros y rate limit."""
        if self._ok(ch, self.LVL["warn"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)
