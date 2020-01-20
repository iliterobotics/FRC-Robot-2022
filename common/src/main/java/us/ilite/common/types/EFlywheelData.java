package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EFlywheelData implements CodexOf<Double> {
    TURRET_AZIMUTH_DEG,
    // other sensor data


    ACTUAL_FLYWHEEL_VELOCITY,
    TARGET_FLYWHEEL_VELOCITY,
    FLYWHEEL_CURRENT

     // Other output data (turret, hood angle servo output, etc)

}
