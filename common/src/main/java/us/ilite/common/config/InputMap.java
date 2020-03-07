package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

import java.lang.annotation.ElementType;

public class InputMap {

    /**
     * Driver
     * ----------
     * Turn axis - right joystick
     * Throttle axis - left joystick
     * Snail mode - R trigger
     * Hanger lock - start
     * Ground track - A
     * Fire power cells - L trigger
     *
     *
     * Operator
     * ----------
     * Intake out - L trigger
     * Intake stow - R trigger
     * Intake bulldoze - X
     * Aim - B
     * Conveyors - L bumper
     * Reverse power cells - R bumper
     * Hang - right joystick
     * Color wheel position - Dpad right
     * Color wheel rotation - Dpad left
     * Near/Far mode (toggle) - Y
     * Manual hood control - Dpad up/down
     * */

    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS,
        HANGER_LOCK = ELogitech310.START,
        GROUND_TRACK = ELogitech310.A_BTN,
        FIRE_POWER_CELLS = ELogitech310.LEFT_TRIGGER_AXIS,

        //NUDGE_SEEK_LEFT = ELogitech310.L_BTN,
        //NUDGE_SEEK_RIGHT = ELogitech310.R_BTN,
        //BEGIN_HANG = ELogitech310.Y_BTN,
        //RELEASE_HANG = ELogitech310.A_BTN,
        //TRACK_CARGO_BTN = ELogitech310.X_BTN,
        //TRACK_TARGET_BTN = ELogitech310.A_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.B_BTN,
        DRIVER_LIMELIGHT_LOCK_BALL = ELogitech310.A_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM = ELogitech310.Y_BTN,
        //DRIVER_LIMELIGHT_LOCK_BALL_TRI = ELogitech310.X_BTN,
        DRIVER_LIMELIGHT_LOCK_BALL_DUAL = ELogitech310.START
        ;
    }

    public static class OPERATOR {
        public static  final ELogitech310
        INTAKE_BULLDOZE = ELogitech310.X_BTN,
        AIM = ELogitech310.B_BTN,

        FAR_MODE = ELogitech310.Y_BTN,
        NEAR_MODE = ELogitech310.A_BTN,
        CLOSE_MODE = ELogitech310.L_BTN,
        MANUAL_HOOD_UP = ELogitech310.DPAD_UP,
        MANUAL_HOOD_DOWN = ELogitech310.DPAD_DOWN,

        // DO NOT CHANGE THESE
        INTAKE_ACTIVATE = ELogitech310.LEFT_TRIGGER_AXIS,
        INTAKE_REVERSE = ELogitech310.R_BTN,
        INTAKE_STOW = ELogitech310.RIGHT_TRIGGER_AXIS,
        RESET_INTAKE_COUNT = ELogitech310.BACK,

        // PUT TEST ITEMS BELOW
        BEGIN_HANG = ELogitech310.LEFT_Y_AXIS,
        REVERSE_HANG = ELogitech310.LEFT_X_AXIS,
        //RELEASE_HANG = ELogitech310.START,
        COLOR_POSITION = ELogitech310.DPAD_RIGHT,
        //FLYWHEEL_AXIS = ELogitech310.START,
        COLOR_ROTATION = ELogitech310.DPAD_LEFT,
        //SHOOT_FLYWHEEL = ELogitech310.A_BTN,
        LIMELIGHT_SEARCH = ELogitech310.RIGHT_X_AXIS
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
                RESET_INTAKE_COUNT = ELogitech310.BACK,
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

    public static class OPERATOR_REFACTOR {
        public static final ELogitech310
                MANUAL_TURRET = ELogitech310.RIGHT_X_AXIS

                ;
    }
}
