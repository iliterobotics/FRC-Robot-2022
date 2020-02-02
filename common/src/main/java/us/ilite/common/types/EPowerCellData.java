package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData implements CodexOf<Double> {

    BREAK_SENSOR_0,
    BREAK_SENSOR_1,
    BREAK_SENSOR_2,

    // etc - 1 per beam break sensor

    DESIRED_H_POWER_PCT,
    CURRENT_H_POWER_PCT,
    DESIRED_V_POWER_PCT,
    CURRENT_V_POWER_PCT,

    DESIRED_INTAKE_POWER_PCT, //Talon
    CURRENT_INTAKE_POWER_PCT,

    DESIRED_INTAKE_STATE,
    DESIRED_ARM_STATE,

    CURRENT_ARM_ANGLE,
    DESIRED_ARM_ANGLE,

    CURRENT_INTAKE_VELOCITY_FT_S,
    DESIRED_INTAKE_VELOCITY_FT_S,

    DESIRED_INDEXING_STATE,
    CURRENT_INDEXING_STATE;
}
