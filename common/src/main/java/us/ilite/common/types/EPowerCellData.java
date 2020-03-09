package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData implements CodexOf<Double> {
    ENTRY_BEAM,
    ENTRY_GATE,
    H_BEAM,
    H_GATE,
    EXIT_BEAM,
    EXIT_GATE,
    NUM_BALLS,

    SET_H_pct,
    SET_V_pct,

    INTAKE_STATE,
    ARM_ANGLE_deg,
    PIVOT_ABSOLUTE_ENCODER_RAW,

    INTAKE_VEL_ft_s,
    SET_INTAKE_VEL_ft_s,

    INTAKE_PIVOT_CURRENT,
    INTAKE_ROLLER_CURRENT,
    SERIALIZER_CURRENT,
    VERTICAL_CURRENT
}
