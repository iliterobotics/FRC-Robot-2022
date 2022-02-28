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
     * Spit - dpad up
     * Hang fwd - right trigger
     * Hang rev - left trigger
     * Hang position lock - start button
     * */

    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS, // Arcade drive
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SNAIL_MODE = ELogitech310.RIGHT_TRIGGER_AXIS, // Lower velocity cap and slower acceleration
        BOOST_MODE = ELogitech310.LEFT_TRIGGER_AXIS, // Experimental Mode with faster acceleration and higher velocity cap
        HOME_TO_DRIVER_STATION = ELogitech310.X_BTN, // Turns the robot towards the driver station
        DRIVER_LIMELIGHT_LOCK_TARGET = ELogitech310.A_BTN, // Limelight target-locks to the goal
        HOME_TO_HUB = ELogitech310.Y_BTN, // Practically target-lock but without limelight
        ACTIVATE_CLIMB = ELogitech310.START // Allows the manipulator to begin the climb
        ;
    }

    public static class OPERATOR {
        public static final ELogitech310
        RETRACT_INTAKE = ELogitech310.RIGHT_TRIGGER_AXIS, // Pulls the intake up
        EXTEND_INTAKE = ELogitech310.LEFT_TRIGGER_AXIS, // Puts the intake down
        SPIN_ROLLERS = ELogitech310.A_BTN, // Spins the rollers inwards
        REVERSE_ROLLERS = ELogitech310.B_BTN, // Spins the rollers outwards
        SPIN_FEEDER = ELogitech310.X_BTN, // Spins the entire feeder system forwards
        REVERSE_FEEDER = ELogitech310.Y_BTN, // Spins the entire feeder system backwards
        RELEASE_BALLS = ELogitech310.DPAD_UP // Spins the feeder and intake system in reverse
        ;

    }

    public static class HANGER {
        public static final ELogitech310
        MANUAL_FWD = ELogitech310.RIGHT_TRIGGER_AXIS,
        MANUAL_REV = ELogitech310.LEFT_TRIGGER_AXIS,
        POSITION_LOCK = ELogitech310.START
        ;
    }

}
