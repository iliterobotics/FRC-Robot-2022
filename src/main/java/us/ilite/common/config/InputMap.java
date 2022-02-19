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
        HOME_TO_DRIVER_STATION = ELogitech310.Y_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.B_BTN;

    }

    public static class OPERATOR {

        ;

    }

}
