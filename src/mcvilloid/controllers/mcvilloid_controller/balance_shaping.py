# src/mcvilloid/controllers/mcvilloid_controller/balance_shaping.py
"""
Post-proceso de las salidas del BalanceController.

Aquí se toma el diccionario de offsets generado por BalanceController.step()
y se aplica un shaping extra, principalmente:

- Recorte específico en tobillos pitch para evitar que el balance empuje
  demasiado cuando el robot se va de espaldas.
- Deja pasar tal cual los offsets de tobillo roll.
- Ignora cualquier otro joint (caderas, rodillas, etc.), manteniendo el
  comportamiento que tenías en la versión previa del controller.
"""


def compute_balance_offsets(
    dt,
    bal,
    pitch,
    roll,
    pitch_f,
    BAL_PITCH_LIM,
    state,
    LOG,
):
    """
    Calcula offsets de balance ya “moldeados” para ser sumados a `target`.

    Parámetros
    ----------
    dt : float
        Paso de tiempo del controlador (segundos). No se usa aún, pero se
        mantiene en la firma por compatibilidad y posible uso futuro.
    bal : BalanceController
        Instancia de BalanceController; se llama a `bal.step(dt, pitch, roll)`.
    pitch : float
        Pitch crudo de IMU (rad). No se usa directamente aquí; se delega a
        `bal.step` y se conserva en la firma por claridad.
    roll : float
        Roll crudo de IMU (rad).
    pitch_f : float
        Pitch filtrado / suavizado, usado para decidir cómo recortar los offsets
        de tobillo pitch cuando el robot se va hacia atrás.
    BAL_PITCH_LIM : float
        Límite simétrico máximo en rad para tobillos pitch en condiciones normales.
    state : str
        Estado actual del FSM de marcha (STAND, PRELEAN, WALK, RECOVER, ...).
        No se usa aún en este shaping, se mantiene por futuras extensiones.
    LOG : logger compatible
        Logger (RateLogger o similar). No se usa aquí pero se conserva en la
        firma para poder loguear en el futuro sin cambiarla.

    Devuelve
    --------
    dict
        Diccionario {joint_name: offset_rad} listo para sumarse a `target[]`.
        Sólo incluye:
          - jXX_ankle_pitch_* con recorte dependiente de pitch_f.
          - jXX_ankle_roll_* tal cual viene de BalanceController.
    """
    # Llamamos al controlador de balance de bajo nivel
    bal_off = bal.step(dt, pitch, roll)
    bal_post = {}

    if not bal_off:
        return bal_post

    for j, off in bal_off.items():
        if "ankle_pitch" in j:
            # Mismo recorte que traías en el controller viejo.
            lim = BAL_PITCH_LIM

            if pitch_f < 0.0:
                # Si se va de espaldas:
                #  - Permitimos plantar (negativo) hasta -lim
                #  - Limitamos MUCHO la dorsiflexión extra (positiva)
                MAX_DORSI = +0.02  # rad (~1.1°)
                off = max(-lim, min(MAX_DORSI, off))
            else:
                # Normal (frente): límite simétrico ±lim
                off = max(-lim, min(lim, off))

            bal_post[j] = off

        elif "ankle_roll" in j:
            # Para roll de tobillo no tocamos nada por ahora,
            # solo lo dejamos pasar si el BalanceController lo da.
            bal_post[j] = off

        # otros joints (caderas, rodillas, etc.) se ignoran,
        # que es lo más parecido a lo que traías en la versión previa.

    return bal_post
