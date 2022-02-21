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
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS, // Arcade drive
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,

        LEFT_AXIS = ELogitech310.LEFT_Y_AXIS, // Tank Drive
        RIGHT_AXIS = ELogitech310.RIGHT_Y_AXIS,

        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS,
        HOME_TO_DRIVER_STATION = ELogitech310.X_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.A_BTN,
        HOME_TO_HUB = ELogitech310.Y_BTN, // Practically target-lock but without limelight

        POSITION_CLIMB = ELogitech310.BACK,
        RUN_CLIMB = ELogitech310.START;
        ;
    }

    public static class OPERATOR {
        public static final ELogitech310
        SHOOT_CARGO = ELogitech310.RIGHT_TRIGGER_AXIS,
        RUN_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS,
        REVERSE_INTAKE = ELogitech310.L_BTN,
        FAR_ANGLE = ELogitech310.Y_BTN,
        RUN_CLIMB = ELogitech310.START,
        MANUAL_CLIMB = ELogitech310.RIGHT_Y_AXIS;
        ;

    }

}
