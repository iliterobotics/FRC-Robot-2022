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
        MID_RUNG = ELogitech310.L_BTN
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
        RELEASE_BALLS = ELogitech310.DPAD_LEFT,
        SHOOT_CARGO = ELogitech310.R_BTN,
        PLACE_CARGO = ELogitech310.L_BTN
        ;
    }

    public static class HANGER {
        public static final ELogitech310
        SPIN_DOUBLE = ELogitech310.RIGHT_TRIGGER_AXIS,
        SPIN_SINGLE = ELogitech310.LEFT_TRIGGER_AXIS,
//        BOOST_MODE = ELogitech310.R_BTN,
//        SNAIL_MODE = ELogitech310.L_BTN,
        HIGH_RUNG = ELogitech310.L_BTN,
        TRAVERSAL_RUNG = ELogitech310.R_BTN,
        BALANCE_CLIMBER = ELogitech310.START,
//        SET_COAST = ELogitech310.BACK,
        CLAMP_DOUBLE = ELogitech310.DPAD_UP,
        RELEASE_DOUBLE = ELogitech310.DPAD_DOWN,
        CLAMP_SINGLE = ELogitech310.Y_BTN,
        RELEASE_SINGLE = ELogitech310.A_BTN,

        // Experimental Inputs for automation
        CLIMB_TO_NEXT_RUNG = ELogitech310.R_BTN
        ;
    }

}
