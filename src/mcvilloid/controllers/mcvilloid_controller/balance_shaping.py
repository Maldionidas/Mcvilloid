# balance_shaping.py

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
    Aplica el controlador de balance y limita sus offsets principalmente en tobillos.

    Devuelve:
      bal_post: dict {joint_name: offset} listo para sumarse a target[]
    """
    bal_off = bal.step(dt, pitch, roll)
    bal_post = {}

    if not bal_off:
        return bal_post

    for j, off in bal_off.items():
        if "ankle_pitch" in j:
            # Mismo recorte que traías en el controller viejo
            lim = BAL_PITCH_LIM

            if pitch_f < 0.0:
                # Si se va de espaldas:
                # - Permitimos plantar (negativo) hasta -lim
                # - Limitamos MUCHO la dorsiflexión extra (positiva)
                MAX_DORSI = +0.02  # rad (~1.1°)
                off = max(-lim, min(MAX_DORSI, off))
            else:
                # Normal (frente): límite simétrico
                off = max(-lim, min(lim, off))

            bal_post[j] = off

        elif "ankle_roll" in j:
            # Para roll de tobillo no tocamos nada por ahora,
            # solo lo dejamos pasar si el BalanceController lo da.
            bal_post[j] = off

        # otros joints (caderas, rodillas, etc.) se ignoran,
        # que es lo más parecido a lo que traías en la versión previa.

    return bal_post
