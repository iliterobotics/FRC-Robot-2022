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
        DRIVER_LIMELIGHT_LOCK_BALL_TRI = ELogitech310.X_BTN,
        DRIVER_LIMELIGHT_LOCK_BALL_DUAL = ELogitech310.START
        ;
    }

    public static class OPERATOR {
        public static  final ELogitech310
        // DO NOT CHANGE THESE
        INTAKE_ACTIVATE = ELogitech310.A_BTN,
        INTAKE_REVERSE = ELogitech310.X_BTN,
        INTAKE_STOW = ELogitech310.B_BTN,
        FIRE_POWER_CELLS = ELogitech310.Y_BTN,

                // PUT TEST ITEMS BELOW
        BEGIN_HANG = ELogitech310.START,
        RELEASE_HANG = ELogitech310.START,
        COLOR_POSITION = ELogitech310.START,
        FLYWHEEL_AXIS = ELogitech310.START,
        COLOR_ROTATION = ELogitech310.START,
        SHOOT_FLYWHEEL = ELogitech310.A_BTN,
        LOWER_ARM = ELogitech310.START
                ;
    }

    // For flywheel tuning, Joystick port 2 (driver's station row 3)
    public static class FLYWHEEL {
        public static final ELogitech310
                HOOD = ELogitech310.COMBINED_TRIGGER_AXIS,
                HOOD_TO_ANGLE = ELogitech310.Y_BTN,
                TEST_FIRE = ELogitech310.L_BTN,
                BASIC_INTAKE = ELogitech310.B_BTN,
                INTAKE_STOW = ELogitech310.A_BTN,
                REVERSE_INTAKE = ELogitech310.X_BTN,
                FLYWHEEL_SPINUP_TEST = ELogitech310.START,
                FLYWHEEL_VELOCITY_10_TEST = ELogitech310.DPAD_UP,
                FLYWHEEL_VELOCITY_20_TEST = ELogitech310.DPAD_RIGHT,
                FLYWHEEL_VELOCITY_30_TEST = ELogitech310.DPAD_DOWN,
                FLYWHEEL_VELOCITY_40_TEST = ELogitech310.DPAD_LEFT,
                FEEDER_SPINUP_TEST = ELogitech310.Y_BTN,
                FLYWHEEL_SPINUP_AXIS = ELogitech310.RIGHT_TRIGGER_AXIS
        ;
    }

    public static class LIMELIGHT {
        public static final ELogitech310
                LIMELIGHT_LOCK_TARGET = ELogitech310.B_BTN,
                LIMELIGHT_LOCK_BALL = ELogitech310.A_BTN,
                LIMELIGHT_LOCK_TARGET_ZOOM = ELogitech310.Y_BTN,
                LIMELIGHT_LOCK_BALL_TRI = ELogitech310.X_BTN,
                LIMELIGHT_LOCK_BALL_DUAL = ELogitech310.START
                        ;
    }
}
