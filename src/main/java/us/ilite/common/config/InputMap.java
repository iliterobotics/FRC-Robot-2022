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
     * Limelight lock target - A button
     * Activate climb - start button
     * Mid-Rung climb - L Button
     */
    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS, // TODO Experiment with increased and decreased acceleration rates for snail and boost mode
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.A_BTN,
        ACTIVATE_CLIMB = ELogitech310.START,
        MID_RUNG = ELogitech310.L_BTN
        ;
    }

    /**
     *
     * Operator
     * ----------
     * Intake retract - R trigger
     * Intake extend - L trigger
     * Roller outwards -  B button
     * Feeder fwd - X button
     * Feeder rev - Y button
     * Release Balls - dpad left
     * Shoot cargo - r button
     * Place cargo - l button
     **/
    public static class OPERATOR {
        public static final ELogitech310
        RETRACT_INTAKE = ELogitech310.RIGHT_TRIGGER_AXIS,
        EXTEND_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS,
        REVERSE_ROLLERS = ELogitech310.B_BTN,
        SPIN_FEEDER = ELogitech310.X_BTN,
        REVERSE_FEEDER = ELogitech310.Y_BTN,
        RELEASE_BALLS = ELogitech310.DPAD_LEFT,
        SHOOT_CARGO = ELogitech310.R_BTN,
        PLACE_CARGO = ELogitech310.L_BTN;
    }

    /**
     * Hanger buttons
     * Spin double - right trigger axis
     * Spin single - left trigger axis
     * High rung - l button
     * Traversal run - r button
     * Clamp double - dpad up
     * Release double - dpad down
     * Clamp single - y button
     * Release single - a button
     * Climb to next rung - r button
     */
    public static class HANGER {
        public static final ELogitech310
        SPIN_DOUBLE = ELogitech310.RIGHT_TRIGGER_AXIS,
        SPIN_SINGLE = ELogitech310.LEFT_TRIGGER_AXIS,
        HIGH_RUNG = ELogitech310.L_BTN,
        TRAVERSAL_RUNG = ELogitech310.R_BTN,
        CLAMP_DOUBLE = ELogitech310.DPAD_UP,
        RELEASE_DOUBLE = ELogitech310.DPAD_DOWN,
        CLAMP_SINGLE = ELogitech310.Y_BTN,
        RELEASE_SINGLE = ELogitech310.A_BTN,
        // Experimental Inputs for automation
        CLIMB_TO_NEXT_RUNG = ELogitech310.R_BTN;
    }

}
