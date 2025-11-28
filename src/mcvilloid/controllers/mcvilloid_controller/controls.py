# controls.py
"""
Manejo de teclado para McVilloid (Webots Keyboard).

Teclas soportadas
-----------------
Marcha:
    - W / w : habilitar marcha (gait_enable = True)
    - S / s : deshabilitar marcha (gait_enable = False)

Dirección:
    - A / a : dirección hacia atrás (walker.dir = -1.0)
    - D / d : dirección hacia adelante (walker.dir = +1.0)

Poses estáticas (levantar pierna con transferencia de peso):
    - F : activar pose en pierna izquierda (side = 'L')
    - G : activar pose en pierna derecha  (side = 'R')
    - H : cancelar pose actual

Zancada base:
    - P : aumentar STRIDE_BASE (limitado por STRIDE_MAX)
    - M : reducir  STRIDE_BASE (limitado por STRIDE_MIN)
"""


def handle_keyboard(
    kb,
    walker,
    pose,
    STRIDE_BASE,
    STRIDE_MIN,
    STRIDE_MAX,
    gait_enable,
    LOG,
    enabled: bool = True,
):
    """
    Lee las teclas del teclado Webots y actualiza varios estados de marcha.

    Parámetros
    ----------
    kb :
        Instancia de Keyboard de Webots (robot.getKeyboard()).
    walker :
        Instancia de Walker encargada de generar offsets de marcha.
    pose : dict
        Diccionario de estado de la FSM de pose (levantar pierna estática).
        Se modifica in-place con campos como:
            - "on":   bool (pose activa o no)
            - "side": 'L' o 'R'
            - "phase": fase interna ("SHIFT", "LIFT", etc., según pose_fsm)
            - "t":   tiempo acumulado en la fase
    STRIDE_BASE : float
        Valor actual de la zancada base (multiplicador de amplitud).
    STRIDE_MIN : float
        Límite inferior permitido para STRIDE_BASE.
    STRIDE_MAX : float
        Límite superior permitido para STRIDE_BASE.
    gait_enable : bool
        Flag global que indica si la marcha está habilitada (FSM principal).
    LOG :
        Logger compatible con RateLogger, usado para imprimir eventos.
    enabled : bool, opcional
        Si es False, se ignoran por completo las teclas (no se modifica nada).

    Devuelve
    --------
    (gait_enable, STRIDE_BASE) : tuple
        El nuevo valor de gait_enable y STRIDE_BASE después de procesar teclas.

    Notas
    -----
    - Si `enabled` es False, la función simplemente devuelve los valores de
      entrada sin leer el teclado.
    - Esta función no avanza ningún estado por sí misma, sólo actualiza flags
      y parámetros que luego usará el controller y el Walker.
    """

    # Si el teclado está deshabilitado por config, no hacemos nada
    if not enabled:
        return gait_enable, STRIDE_BASE

    key = kb.getKey()
    while key != -1:
        # --- Marcha ON/OFF ---
        if key in (ord('W'), ord('w')):
            gait_enable = True
            LOG.info("gait", "gait_enable = True")

        elif key in (ord('S'), ord('s')):
            gait_enable = False
            LOG.info("gait", "gait_enable = False")

        # --- Dirección ---
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
