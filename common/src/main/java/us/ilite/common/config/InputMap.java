package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

public class InputMap {

    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SUB_WARP_AXIS = ELogitech310.LEFT_TRIGGER_AXIS,

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
        LOWER_ARM = ELogitech310.START
                ;
    }

}
