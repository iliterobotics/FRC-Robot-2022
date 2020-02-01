package us.ilite.common.types.sensor;

import com.flybotix.hfr.codex.CodexOf;
import edu.wpi.first.wpilibj.util.Color;

public enum EColorData implements CodexOf<Double> {
    SENSED_COLOR,
    POSITION_CONTROL_INPUT,
    ROTATION_CONTROL_INPUT,
    COLOR_WHEEL_MOTOR_STATE;


    public enum EColor {
        RED( new Color( 0.561, 0.232, 0.114 ) ),
        BLUE( new Color( 0.143, 0.427, 0.429 ) ),
        GREEN( new Color( 0.197, 0.561, 0.240 ) ),
        YELLOW( new Color( 0.361, 0.524, 0.113 ) ),
        NONE( new Color( 0, 0, 0 ) );

        Color color;
        EColor(Color c) {
            color = c;
        }
    }

    public enum EInput {
        POSITIVE,
        NEGATIVE;
    }
    public enum EMotorState {
        ON,
        OFF;
    }

}
