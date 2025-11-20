# controls.py
"""
Manejo de teclado para McVilloid:
- W/S: habilitar/deshabilitar marcha
- A/D: dirección (atrás / adelante)
- F/G/H: poses estáticas (levantar pierna)
- P/M: ajustar zancada base
"""

def handle_keyboard(kb, walker, pose,
                    STRIDE_BASE, STRIDE_MIN, STRIDE_MAX,
                    gait_enable, LOG):
    """
    Lee todas las teclas pendientes y actualiza:
      - gait_enable
      - walker.dir
      - pose
      - STRIDE_BASE (y se lo setea al walker)
    """
    key = kb.getKey()
    while key != -1:
        if key in (ord('W'), ord('w')):
            gait_enable = True
            LOG.info("gait", "gait_enable = True")

        elif key in (ord('S'), ord('s')):
            gait_enable = False
            LOG.info("gait", "gait_enable = False")

        elif key in (ord('A'), ord('a')):  # backward
            walker.dir = -1.0
            LOG.info("gait", "Direction: BACKWARD")

        elif key in (ord('D'), ord('d')):  # forward
            walker.dir = +1.0
            LOG.info("gait", "Direction: FORWARD")

        # --- POSE estática: levantar pierna ---
        elif key == ord('F'):  # levantar pierna IZQ
            pose.update({"on": True, "side": "L", "phase": "SHIFT", "t": 0.0})
            walker.toggle(False)
            walker.set_global_gain(0.0)
            LOG.info("gait", "[POSE] Levantar IZQ: SHIFT")

        elif key == ord('G'):  # levantar pierna DER
            pose.update({"on": True, "side": "R", "phase": "SHIFT", "t": 0.0})
            walker.toggle(False)
            walker.set_global_gain(0.0)
            LOG.info("gait", "[POSE] Levantar DER: SHIFT")

        elif key == ord('H'):  # cancelar pose
            pose.update({"on": False, "phase": "IDLE", "t": 0.0})
            LOG.info("gait", "[POSE] cancelada")

        # --- Ajuste de zancada base ---
        elif key == ord('P'):  # aumentar zancada base
            STRIDE_BASE = min(STRIDE_MAX, STRIDE_BASE + 0.05)
            LOG.info("gait", f"[STRIDE] base -> {STRIDE_BASE:.2f}")
            walker.set_stride_base(STRIDE_BASE)

        elif key == ord('M'):  # reducir zancada base
            STRIDE_BASE = max(STRIDE_MIN, STRIDE_BASE - 0.05)
            LOG.info("gait", f"[STRIDE] base -> {STRIDE_BASE:.2f}")
            walker.set_stride_base(STRIDE_BASE)

        key = kb.getKey()

    return gait_enable, STRIDE_BASE
