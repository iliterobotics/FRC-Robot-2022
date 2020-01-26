package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EShooterSystemData implements CodexOf<Double> {
    CURRENT_FLYWHEEL_STATE,
    TARGET_FLYWHEEL_STATE,
    CURRENT_FLYWHEEL_VELOCITY,
    TARGET_FLYWHEEL_VELOCITY,
    CURRENT_LIMELIGHT_TARGET,
    TARGET_LIMELIGHT_TARGET,
    CURRENT_TURRET_VELOCITY,
    TARGET_TURRET_VELOCITY,
    CURRENT_HOOD_ANGLE,
    TARGET_HOOD_ANGLE,
    CURRENT_ACCELERATOR_STATE,
    TARGET_ACCELERATOR_STATE,
    CURRENT_TURRET_MODE,
    TARGET_TURRET_MODE
}
