package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EFlywheelData implements CodexOf<Double> {
    CURRENT_FLYWHEEL_STATE,
    TARGET_FLYWHEEL_STATE,
    CURRENT_FLYWHEEL_VELOCITY;
}
