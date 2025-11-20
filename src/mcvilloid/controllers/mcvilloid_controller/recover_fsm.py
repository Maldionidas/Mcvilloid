def recover_step(dt,
                 pitch_f,
                 roll_f,
                 gy_val,
                 gyro_ok,
                 target,
                 base_pose,
                 rec_pose,
                 recover_timer,
                 LOG):
    recover_timer += dt

    # --- 0) Detección simple de dirección de caída ------------------------
    # Umbrales en radianes
    BACK_PITCH = -0.16   # se va claramente de nalgas
    FWD_PITCH  = +0.16   # se va de cara

    falling_back  = (pitch_f < BACK_PITCH)
    falling_front = (pitch_f > FWD_PITCH)

    # --- 1) Postura base según tipo de caída -----------------------------
    # Ajustamos posturas para que no sean TAN sentadas
    if falling_back:
        # Caída hacia atrás: torso adelante, rodilla moderada,
        # un poco de dorsiflexión
        REC_HIP_ABS  = -0.30
        REC_KNEE_ABS = +0.24
        REC_ANK_ABS  = -0.04
    elif falling_front:
        # Caída hacia delante: torso un poco atrás,
        # rodilla algo flexionada, tobillo leve plantar
        REC_HIP_ABS  = -0.08
        REC_KNEE_ABS = +0.2
        REC_ANK_ABS  = -0.02
    else:
        # Zona intermedia: semi-sentadilla suave, torso ligeramente adelante
        REC_HIP_ABS  = -0.20
        REC_KNEE_ABS = +0.22
        REC_ANK_ABS  = -0.04

    # Aplicar postura base
    for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
        if hip in target:
            target[hip] = REC_HIP_ABS

    for knee in ("j03_knee_pitch_l", "j09_knee_pitch_r"):
        if knee in target:
            target[knee] = REC_KNEE_ABS

    for ank in ("j04_ankle_pitch_l", "j10_ankle_pitch_r"):
        if ank in target:
            target[ank] = REC_ANK_ABS

    # Limitamos ankle roll como ya hacías
    for j in ("j05_ankle_roll_l", "j11_ankle_roll_r"):
        if j in target:
            target[j] = max(-0.04, min(0.04, target[j]))

    # --- 2) PD sobre pitch (una sola convención de signo) ----------------
    # Queremos el torso un pelín adelantado
    pitch_target = +0.02

    Kp, Kd = 0.22, 0.05
    err = pitch_target - pitch_f
    u = Kp * err
    if gyro_ok:
        u -= Kd * gy_val

    # Saturación suave
    u = max(-0.15, min(0.15, u))

    if LOG:
        LOG.info(
            "recover",
            f"[REC] t={recover_timer:.2f}s pitch_f={pitch_f:+.3f} "
            f"err={err:+.3f} u={u:+.3f}"
        )

    # --- 3) Mapear u a articulaciones ------------------------------------
    # Regla única:
    # - hip_pitch más NEGATIVO = torso más ADELANTE
    #   * si u > 0 (caída atrás) => queremos más hip negativa => restamos
    #   * si u < 0 (caída delante) => queremos hip más positiva => sumamos
    #
    # - rodilla: pequeño ajuste, sin exagerar:
    #   * u > 0 => un poco menos flexión (más extendida) para empujar COM adelante
    #   * u < 0 => un poco más flexión para ir abajo/atrás
    #
    k_hip  = 0.6
    k_knee = 0.25

    # HIPS
    for hip in ("j00_hip_pitch_l", "j06_hip_pitch_r"):
        if hip in target:
            target[hip] += -k_hip * u
            target[hip] = max(-0.45, min(0.20, target[hip]))

    # KNEES
    for knee in ("j03_knee_pitch_l", "j09_knee_pitch_r"):
        if knee in target:
            target[knee] += -k_knee * u
            target[knee] = max(0.08, min(0.55, target[knee]))

    # Tobillo pitch: de momento SIN término proporcional a u,
    # nos quedamos solo con REC_ANK_ABS para no meter más ruido.

    return recover_timer
