package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

public class InputMap {

    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SUB_WARP_AXIS = ELogitech310.LEFT_TRIGGER_AXIS,
        NUDGE_SEEK_LEFT = ELogitech310.L_BTN,
        NUDGE_SEEK_RIGHT = ELogitech310.R_BTN,
        BEGIN_HANG = ELogitech310.Y_BTN,
        RELEASE_HANG = ELogitech310.A_BTN,
        TRACK_CARGO_BTN = ELogitech310.X_BTN,
        TRACK_TARGET_BTN = ELogitech310.A_BTN,

        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.B_BTN,
        DRIVER_LIMELIGHT_LOCK_BALL = ELogitech310.A_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM = ELogitech310.Y_BTN,
        DRIVER_LIMELIGHT_LOCK_BALL_DUAL = ELogitech310.START,
        DRIVER_LIMELIGHT_LOCK_BALL_TRI = ELogitech310.X_BTN,

        TRACK_HATCH_BTN = ELogitech310.B_BTN;

    }

    public static class OPERATOR {
        public static final ELogitech310
        OPERATOR_POSITION_CONTROL = ELogitech310.R_BTN,
        OPERATOR_ROTATION_CONTROL = ELogitech310.R_BTN,
        SHOOT_FLYWHEEL = ELogitech310.A_BTN,
        LOCK_TARGET = ELogitech310.A_BTN,
        LOCK_CELL = ELogitech310.B_BTN,
        LOCK_TARGET_ZOOM = ELogitech310.Y_BTN,
        INTAKE = ELogitech310.X_BTN,
        REVERSE_INTAKE = ELogitech310.A_BTN,
        LOWER_ARM = ELogitech310.Y_BTN,
        HIGHER_ARM = ELogitech310.B_BTN;
    }

}
