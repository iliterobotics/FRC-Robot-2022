package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

import java.lang.annotation.ElementType;

public class InputMap {

    /**
     * Driver
     * ----------
     * Turn axis - right joystick x-axis
     * Throttle axis - left joystick y-axis
     * Snail mode - right trigger
     * Boost mode - left trigger
     * Activate climb - start button
     * Home to DS - X button
     * Home to hub - Y button
     * Target-Lock - A button
     *
     * Operator
     * ----------
     * Intake extend - L trigger
     * Intake retract - R trigger
     * Roller inwards - A button
     * Roller outwards - B button
     * Feeder fwd - X button
     * Feeder rev - Y button
     * Place cargo - left bumper
     * Shoot cargo - right bumper
     * Spit - dpad up
     * Hang fwd - right trigger
     * Hang rev - left trigger
     * Hang position lock - start button
     * Lock fwd - right bumper
     * Lock rev - left bumper
     * Manual balls up - dpad up
     * Manual balls down - dpad down
     * */

    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS, // TODO Experiment with increased and decreased acceleration rates for snail and boost mode
        BOOST_MODE = ELogitech310.LEFT_TRIGGER_AXIS,
        HOME_TO_DRIVER_STATION = ELogitech310.X_BTN,
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.A_BTN,
        ACTIVATE_CLIMB = ELogitech310.START,
        MANUAL_REV_SLOW = ELogitech310.Y_BTN,
        MANUAL_FWD_SLOW = ELogitech310.B_BTN
        ;
    }

    public static class OPERATOR {
        public static final ELogitech310
        RETRACT_INTAKE = ELogitech310.RIGHT_TRIGGER_AXIS,
        EXTEND_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS,
        SPIN_ROLLERS = ELogitech310.A_BTN,
        REVERSE_ROLLERS = ELogitech310.B_BTN,
        SPIN_FEEDER = ELogitech310.X_BTN,
        REVERSE_FEEDER = ELogitech310.Y_BTN,
//        RELEASE_BALLS = ELogitech310.DPAD_LEFT,
//        SHOOT_CARGO = ELogitech310.R_BTN,
//        PLACE_CARGO = ELogitech310.L_BTN,
//        MANUAL_BALLS_UP = ELogitech310.DPAD_UP,
//        MANUAL_BALLS_DOWN = ELogitech310.DPAD_DOWN
        TOP_CLAMPED = ELogitech310.DPAD_UP,
        TOP_RELEASED = ELogitech310.DPAD_LEFT,
        BOTTOM_CLAMPED = ELogitech310.DPAD_RIGHT,
        BOTTOM_RELEASED = ELogitech310.DPAD_DOWN
        ;
    }

    public static class HANGER {
        public static final ELogitech310
        MANUAL_FWD_FULL = ELogitech310.RIGHT_TRIGGER_AXIS,
        MANUAL_FWD_HALF_SPEED = ELogitech310.A_BTN,
        MANUAL_REV_FULL = ELogitech310.LEFT_TRIGGER_AXIS,
        MANUAL_REV_HALF_SPEED = ELogitech310.X_BTN,
        TOP_CLAMPED = ELogitech310.DPAD_UP,
        TOP_RELEASED = ELogitech310.DPAD_DOWN,
        BOTTOM_CLAMPED = ELogitech310.DPAD_RIGHT,
        BOTTOM_RELEASED = ELogitech310.DPAD_DOWN,
        POSITION_LOCK = ELogitech310.START
        ;
    }

}
