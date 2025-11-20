# src/mcvilloid/controllers/mcvilloid_controller/safety_limits.py
import time
from typing import Dict, Any, Tuple

def clamp_ctx_init() -> Dict[str, Any]:
    """
    Contexto para clamps:
      - último log por joint
      - temporizador de clamp en tobillos
    """
    return {
        "last_clamp_log": {},
        "ankle_clamp_timer": 0.0,
        "ankle_clamp_hit": False,
    }

def rclamp_with_flag(
    j: str,
    x: float,
    lim: float,
    ctx: Dict[str, Any],
    LOG,
    cooldown_s: float = 0.25,
) -> float:
    """
    Igual que tu rclamp_with_flag original, pero usando un contexto
    en vez de nonlocal.

    - Hace clamp simétrico ±lim
    - Loggea con cooldown
    - Si el joint es de tobillo, marca ankle_clamp_hit en el contexto
    """
    lim = abs(lim)
    clamped = max(-lim, min(lim, x))
    if clamped != x:
        now = time.monotonic()
        last = ctx["last_clamp_log"].get(j, -1e9)
        if now - last >= cooldown_s:
            LOG.info("clamp", f"[CLAMP] {j}: {x:+.3f} -> {clamped:+.3f} (lim ±{lim:.2f})")
            ctx["last_clamp_log"][j] = now

        if "ankle_" in j:
            ctx["ankle_clamp_hit"] = True

    return clamped

def update_ankle_clamp_state(
    state: str,
    dt: float,
    CLAMP_TAU: float,
    ctx: Dict[str, Any],
    LOG,
) -> Tuple[str, float | None]:
    """
    Aplica el decaimiento del clamp de tobillos y decide si hay que ir a RECOVER.

    Devuelve:
      - nuevo_state
      - nuevo_recover_timer (0.0 si entra a RECOVER, None si no cambia)
    """
    DECAY = 0.20  # s^-1 (igual que antes)

    if ctx["ankle_clamp_hit"]:
        ctx["ankle_clamp_timer"] += dt
    else:
        ctx["ankle_clamp_timer"] = max(0.0, ctx["ankle_clamp_timer"] - DECAY * dt)

    # Consumimos el flag para el siguiente ciclo
    ctx["ankle_clamp_hit"] = False

    if state == "WALK" and ctx["ankle_clamp_timer"] >= CLAMP_TAU:
        LOG.info("gait", "[RECOVER] clamp sostenido en tobillo")
        return "RECOVER", 0.0

    return state, None
