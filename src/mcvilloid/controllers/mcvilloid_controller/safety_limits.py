# src/mcvilloid/controllers/mcvilloid_controller/safety_limits.py
"""
Lógica de clamps de seguridad para articulaciones.
--------------------------------------------------
Aquí se gestionan:

- Clamps de referencia por tipo de joint (se hace en joint_limits.py llamando a
  rclamp_with_flag).
- Detección de "clamp sostenido" en tobillos: si durante un cierto tiempo
  (CLAMP_TAU) los tobillos requieren clamp continuamente, se dispara un
  RECOVER desde la marcha.

Se usa un contexto (dict) compartido entre llamadas para llevar timers
y cooldowns de log.
"""

import time
from typing import Dict, Any, Tuple


def clamp_ctx_init() -> Dict[str, Any]:
    """
    Inicializa el contexto de clamps.

    Devuelve un diccionario con:
      - "last_clamp_log": dict[joint_name -> timestamp último log de clamp]
      - "ankle_clamp_timer": tiempo acumulado con clamp en tobillos (s)
      - "ankle_clamp_hit": flag de "se clamp-eó un tobillo en este ciclo"
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
    Clamp simétrico con log y flag de tobillo.

    Parámetros
    ----------
    j : str
        Nombre de la articulación (ej. "j04_ankle_pitch_l").
    x : float
        Valor de referencia que queremos clamp-ear.
    lim : float
        Límite absoluto (se usará ±lim).
    ctx : dict
        Contexto mutable generado por clamp_ctx_init().
    LOG :
        Logger con método .info(ch, msg).
    cooldown_s : float
        Tiempo mínimo entre logs para el mismo joint.

    Comportamiento
    --------------
    - Aplica clamp simétrico: ref_clamped = clamp(x, -lim, +lim).
    - Si clamp-ea (ref_clamped != x) loggea un mensaje con cooldown.
    - Si el joint es de tobillo ("ankle_"), marca ctx["ankle_clamp_hit"] = True.

    Devuelve
    --------
    float
        Valor clamp-eado.
    """
    lim = abs(lim)
    clamped = max(-lim, min(lim, x))

    if clamped != x:
        now = time.monotonic()
        last = ctx["last_clamp_log"].get(j, -1e9)

        # Log con cooldown por joint
        if now - last >= cooldown_s:
            LOG.info("clamp", f"[CLAMP] {j}: {x:+.3f} -> {clamped:+.3f} (lim ±{lim:.2f})")
            ctx["last_clamp_log"][j] = now

        # Si es tobillo, marcamos para el detector de clamp sostenido
        if "ankle_" in j:
            ctx["ankle_clamp_hit"] = True

    return clamped


def update_ankle_clamp_state(
    state: str,
    dt: float,
    CLAMP_TAU: float,
    clamp_ctx: Dict[str, Any],
    LOG,
) -> Tuple[str, float | None]:
    """
    Actualiza el estado del "clamp sostenido en tobillo" y decide si hay RECOVER.

    Parámetros
    ----------
    state : str
        Estado FSM global actual ("STAND", "WALK", "RECOVER", ...).
    dt : float
        Paso de simulación (s).
    CLAMP_TAU : float
        Tiempo umbral (s) de clamp sostenido en tobillos para disparar RECOVER.
    ctx : dict
        Contexto de clamps (ver clamp_ctx_init()).
    LOG :
        Logger con método .info(ch, msg).

    Lógica
    ------
    - Si en este ciclo hubo clamp en algún tobillo (ctx["ankle_clamp_hit"]):
        * Incrementa ctx["ankle_clamp_timer"].
    - Si NO hubo clamp:
        * Decrementa el timer con una constante DECAY (no baja de 0).
    - Si estamos en WALK y ankle_clamp_timer ≥ CLAMP_TAU:
        * Loggea el evento.
        * Cambia el estado a "RECOVER".
        * Reinicia el recover_timer a 0.0 (lo devolvemos como segundo valor).

    Devuelve
    --------
    (nuevo_state, nuevo_recover_timer)
      - nuevo_state : str
      - nuevo_recover_timer : float | None
          0.0 si se entra a RECOVER, None si no hay cambio en RECOVER.
    """
    DECAY = 0.20  # s^-1 (velocidad a la que "olvida" clamp cuando ya no hay)

    if clamp_ctx["ankle_clamp_hit"]:
        clamp_ctx["ankle_clamp_timer"] += dt
    else:
        clamp_ctx["ankle_clamp_timer"] = max(0.0, clamp_ctx["ankle_clamp_timer"] - DECAY * dt)

    # Consumimos el flag para el siguiente ciclo
    clamp_ctx["ankle_clamp_hit"] = False

    if state == "WALK" and clamp_ctx["ankle_clamp_timer"] >= CLAMP_TAU:
        LOG.info("gait", "[RECOVER] clamp sostenido en tobillo")
        return "RECOVER", 0.0

    return state, None
