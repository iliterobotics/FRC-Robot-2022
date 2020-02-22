package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData implements CodexOf<Double> {
    ENTRY_BEAM,
    H_BEAM,
    EXIT_BEAM,

    SET_H_pct,
    SET_V_pct,

    INTAKE_STATE,
    ARM_ANGLE_deg,
    PIVOT_ABSOLUTE_ENCODER_RAW,

    INTAKE_VEL_ft_s,
    SET_INTAKE_VEL_ft_s,

    CURRENT_AMOUNT_OF_SENSORS_BROKEN,
    DESIRED_AMOUNT_OF_SENSORS_BROKEN,

    ALL_BEAMS_BROKEN,

    INTAKE_PIVOT_CURRENT,
    INTAKE_ROLLER_CURRENT,
    SERIALIZER_CURRENT,
    VERTICAL_CURRENT
}
