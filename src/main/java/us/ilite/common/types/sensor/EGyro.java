package us.ilite.common.types.sensor;

import com.flybotix.hfr.codex.CodexOf;

public enum EGyro implements CodexOf<Double> {

    HEADING_DEGREES, PITCH_DEGREES, ROLL_DEGREES,
    YAW_DEGREES,
    ACCEL_X, ACCEL_Y, ACCEL_Z,
    YAW_OMEGA_DEGREES

}
