package us.ilite.robot.hardware;

import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.lib.util.Utils;

public class ContinuousRotationServo extends Servo {
    private static final double OFF = 0.5;
    private boolean mIsInverted = false;
    private double mOutputValue = OFF;
    private double mLastValue = 0.0d;

    /**
     * Constructor.<br>
     *
     * <p>By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br> By default
     * {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
     *                the MXP port
     */
    public ContinuousRotationServo(int channel) {
        super(channel);
    }

    public ContinuousRotationServo inverted(boolean pIsInverted) {
        mIsInverted = pIsInverted;
        return this;
    }

    public double getRawOutputValue() {
        return mOutputValue;
    }

    public double getLastValue() {
        return mLastValue;
    }

    /**
     * Set the servo position.
     *
     * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     *
     * @param value Position from 0.0 to 1.0.
     */
    public void setServo(double value) {
        // -1.0 --> 0
        // 0 --> 0.5
        // 1.0 --> 1.0
        if(Double.isNaN(value)) {
            mOutputValue = OFF;
            mLastValue = 0.0;
        } else {
            mLastValue = value;
            mOutputValue = (Utils.clamp(value, -1.0, 1.0) * (mIsInverted ? -1.0 : 1.0)/2.0 + 0.5);
        }
        set(mOutputValue);
    }
}