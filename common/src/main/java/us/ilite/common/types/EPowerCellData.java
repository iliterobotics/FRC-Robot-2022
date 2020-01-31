package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData  implements CodexOf<Double> {

    BREAK_SENSOR_0,
    BREAK_SENSOR_1,
    BREAK_SENSOR_2,

    // etc - 1 per beam break sensor

    DESIRED_CONVEYOR_TWO_POWER_PCT,, //Neo
    DESIRED_SERLIALIZER_POWER_PCT, //Talon
    DESIRED_CONVEYOR_POWER_PCT, //Talon

    CURRENT_CONVEYOR_TWO_POWER_PCT,
    CURRENT_SERIALIZER_POWER_PCT,
    CURRENT_CONVEYOR_POWER_PCT,

    CURRENT_POWERCELL_STATE,
    DESIRED_INTAKE_STATE,

    CURRENT_ARM_STATE,
    DESIRED_ARM_STATE;
}
