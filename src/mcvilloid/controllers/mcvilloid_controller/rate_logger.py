# src/mcvilloid/controllers/mcvilloid_controller/rate_logger.py

import time

class RateLogger:
    LVL = {"debug": 10, "info": 20, "warn": 30}

    def __init__(self, cfg, now_fn=None, robot_name="???"):
        self.enabled  = bool(cfg.get("enabled", True))
        self.level    = self.LVL.get(str(cfg.get("level", "info")).lower(), 20)
        self.rate_hz  = float(cfg.get("rate_hz", 10.0))
        self.period   = 1.0 / max(1e-6, self.rate_hz)
        self.channels = {k: bool(v) for k, v in cfg.get("channels", {}).items()}
        self._last    = {}
        self._now     = now_fn or time.monotonic
        self.tag      = f"[{robot_name}]"

    def _ok(self, ch, lvl):
        if not self.enabled:
            return False
        if not self.channels.get(ch, True):
            return False
        if lvl < self.level:
            return False
        t = self._now()
        if t - self._last.get(ch, -1e9) < self.period:
            return False
        self._last[ch] = t
        return True

    def debug(self, ch, msg):
        if self._ok(ch, self.LVL["debug"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)

    def info(self, ch, msg):
        if self._ok(ch, self.LVL["info"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)

    def warn(self, ch, msg):
        if self._ok(ch, self.LVL["warn"]):
            print(f"{self.tag}[{ch}] {msg}", flush=True)
