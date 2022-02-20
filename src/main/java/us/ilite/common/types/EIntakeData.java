package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EIntakeData implements CodexOf<Double> {
    ARM_STATE,
    RETRACT,
    EXTEND,

    ROLLER_VEL_ft_s,
    SET_ROLLER_VEL_ft_s,
    CURRENT_ROLLER_RPM,
    SET_ROLLER_RPM,
    DESIRED_PCT,
    INTAKE_SUPPLY_CURRENT,
    INTAKE_STATOR_CURRENT,
    STATE;

}
