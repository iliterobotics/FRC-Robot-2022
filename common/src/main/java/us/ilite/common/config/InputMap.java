package us.ilite.common.config;


import static us.ilite.common.types.input.ELogitech310.*;

import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.input.ELogitechAttack3;

public class InputMap {
    public static class DRIVER {
        public static final ELogitech310
        TURN_AXIS = ELogitech310.RIGHT_X_AXIS,
        THROTTLE_AXIS = ELogitech310.LEFT_Y_AXIS,
        SUB_WARP_AXIS = ELogitech310.LEFT_TRIGGER_AXIS,
        FLYWHEEL_AXIS = ELogitech310.LEFT_TRIGGER_AXIS
                ;
    }

    public static class OPERATOR {
        public static final ELogitech310
        SHOOT = ELogitech310.RIGHT_TRIGGER_AXIS,
        LOCK_TARGET = ELogitech310.A_BTN,
        LOCK_CELL = ELogitech310.B_BTN,
        LOCK_TARGET_ZOOM = ELogitech310.Y_BTN;
    }

}
