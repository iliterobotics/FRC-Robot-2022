package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EIntakeData implements CodexOf<Double> {
    ARM_STATE,

    ROLLER_VEL_ft_s,
    SET_ROLLER_VEL_ft_s,
    CURRENT_ROLLER_RPM,
    SET_ROLLER_RPM,
    ROLLER_PCT,
    DESIRED_ROLLER_pct,
    INTAKE_SUPPLY_CURRENT,
    INTAKE_STATOR_CURRENT,
    COMPRESSOR_PSI,
    ROLLER_STATE,
    PNEUMATIC_STATE;

}
