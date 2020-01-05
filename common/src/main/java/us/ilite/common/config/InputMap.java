package us.ilite.common.config;


import us.ilite.common.types.input.ELogitech310;

public class InputMap {
    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SUB_WARP_AXIS = ELogitech310.RIGHT_TRIGGER_AXIS,

        NUDGE_SEEK_LEFT = ELogitech310.L_BTN,
        NUDGE_SEEK_RIGHT = ELogitech310.R_BTN,

        TRACK_CARGO_BTN = ELogitech310.X_BTN,
        TRACK_TARGET_BTN = ELogitech310.A_BTN,
        TRACK_HATCH_BTN = ELogitech310.B_BTN


        ;

    }

    public static class OPERATOR {

    }


}
