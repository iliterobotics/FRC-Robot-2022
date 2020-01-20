package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData  implements CodexOf<Double> {

    BREAK_SENSOR_0,
    BREAK_SENSOR_1,
    BREAK_SENSOR_2,

    // etc - 1 per beam break sensor

    DESIRED_INTAKE_POWER_PCT, //Neo
    DESIRED_SERLIALIZER_POWER_PCT, //Talon
    DESIRED_CONVEYOR_POWER_PCT, //Talon

    CURRENT_POWERCELL_STATE;
}
