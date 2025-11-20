# src/mcvilloid/controllers/mcvilloid_controller/telemetry.py

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

    Devuelve:
        last_log_t actualizado
    """
    if not log_cfg.get("enabled", True):
        return last_log_t

    rate = float(log_cfg.get("rate_hz", 10) or 0.0)
    if rate <= 0.0:
        return last_log_t

    period = 1.0 / rate
    if (t - last_log_t) < period:
        return last_log_t

    # Log de gait (state + gain)
    LOG.info(
        "gait",
        f"state={state} gain={getattr(walker, '_global_gain', 1.0):.2f} | pitch={pitch:+.3f}",
    )

    # Log de IMU + algunas juntas
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

    # Log de GPS si existe
    if gps:
        x, y, z = gps.getValues()
        LOG.info("gps", f"[GPS] x={x:.3f} z={z:.3f}")

    return t
