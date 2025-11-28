# src/mcvilloid/controllers/mcvilloid_controller/telemetry.py
"""
Telemetría periódica del gait / IMU / GPS.
-----------------------------------------

Este módulo concentra el logging de alto nivel para depurar la marcha:

- Manda un log de gait por canal "gait" con:
    * estado global (state)
    * ganancia del walker (_global_gain)
    * pitch actual

- Imprime una línea tipo consola con:
    * tiempo simulado
    * pitch / roll
    * contador de pasos
    * un subconjunto de juntas (sample_joints) con sus valores target

- Si hay GPS disponible:
    * loggea posición (x, z) por canal "gps"

La frecuencia de log se controla desde params.json en la sección "logging":
    "logging": {
        "enabled": true,
        "rate_hz": 5,
        ...
    }
"""


def periodic_log(
    t,
    last_log_t,
    log_cfg,
    LOG,
    state,
    walker,
    pitch,
    roll,
    step_count,
    sample_joints,
    target,
    gps,
):
    """
    Hace el log periódico del estado del gait + IMU + algunas juntas.

    Parámetros
    ----------
    t : float
        Tiempo actual de simulación (segundos).
    last_log_t : float
        Último instante en que se hizo log periódico.
    log_cfg : dict
        Config de logging desde params["logging"], se usan:
          - "enabled": bool
          - "rate_hz": float (frecuencia de log)
    LOG :
        Instancia de RateLogger (o similar) con métodos .info(ch, msg).
    state : str
        Estado global del gait ("STAND", "PRELEAN", "WALK", "RECOVER", ...).
    walker :
        Instancia del Walker; se lee walker._global_gain para diagnóstico.
    pitch : float
        Pitch actual (rad), sin filtrar (directo de IMU update en controller).
    roll : float
        Roll actual (rad).
    step_count : int
        Contador de iteraciones / pasos de control (no de pasos de marcha).
    sample_joints : list[str]
        Lista de nombres de juntas a muestrear en el print (ej. rodillas/tobillos).
    target : dict[str, float]
        Referencias actuales de posición por joint (las que se mandarán a motores).
    gps :
        Dispositivo GPS de Webots (o None). Si existe, se loguean x,z.

    Devuelve
    --------
    float
        Nuevo valor de last_log_t (t si se hizo log, o el que ya estaba).
    """
    # Si el logging está deshabilitado por config, no hacemos nada
    if not log_cfg.get("enabled", True):
        return last_log_t

    # Frecuencia deseada de log
    rate = float(log_cfg.get("rate_hz", 10) or 0.0)
    if rate <= 0.0:
        return last_log_t

    period = 1.0 / rate
    if (t - last_log_t) < period:
        # Aún no toca log
        return last_log_t

    # 1) Log de gait (estado + gain + pitch)
    LOG.info(
        "gait",
        f"state={state} gain={getattr(walker, '_global_gain', 1.0):.2f} "
        f"| pitch={pitch:+.3f}",
    )

    # 2) Log de IMU + algunas juntas “representativas”
    sj = ", ".join(
        f"{j}={target.get(j, 0.0):+.3f}"
        for j in sample_joints
        if j in target
    )
    print(
        f"[IMU] t={t:6.2f}s pitch={pitch:+.3f} roll={roll:+.3f} "
        f"steps={step_count} | {sj}",
        flush=True,
    )

    # 3) Log de GPS si existe
    if gps:
        x, y, z = gps.getValues()
        LOG.info("gps", f"[GPS] x={x:.3f} z={z:.3f}")

    # Actualizamos el timestamp de último log
    return t
