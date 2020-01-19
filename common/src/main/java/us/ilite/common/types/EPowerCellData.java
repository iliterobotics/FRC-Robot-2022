package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EPowerCellData  implements CodexOf<Double> {

    BREAK_SENSOR_0,
    BREAK_SENSOR_1,
    // etc - 1 per beam break sensor

    DESIRED_INTAKE_POWER_PCT,
    DESIRED_SERLIALIZER_POWER_PCT,
    DESIRED_CONVEYOR_POWER_PCT,

    CURRENT_POWERCELL_STATE,

    BEAM_BREAKER_STATE;
}
