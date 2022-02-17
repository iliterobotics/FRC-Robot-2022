package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EIntakeData implements CodexOf<Double> {
    ENTRY_BEAM,
    ENTRY_GATE,
    H_BEAM,
    H_GATE,
    EXIT_BEAM,
    EXIT_GATE,
    NUM_BALLS,

    SET_CONVEYOR_pct,
    CONVEYOR_pct,

    LEFT_PNEUMATIC_STATE,
    RIGHT_PNEUMATIC_STATE,

    ROLLER_VEL_ft_s,
    SET_ROLLER_VEL_ft_s,
    INTAKE_STATE,
    ARM_ANGLE_deg,
    INTAKE_VEL_ft_s,
    SET_INTAKE_VEL_ft_s,
    SET_V_pct,
    SET_H_pct;

}
