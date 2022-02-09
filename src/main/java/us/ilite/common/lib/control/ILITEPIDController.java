package us.ilite.common.lib.control;

import us.ilite.common.lib.util.Utils;
import us.ilite.robot.hardware.Clock;

public class ILITEPIDController {
    private boolean mContinuous = false;

    private double mPreviousTime;
    private double mCurrentTime;

    private ProfileGains mProfileGains;

    private double mMaximumOutput = 1.0;
    private double mMinimumOutput = -1.0;
    private double mMaximumInput = 0.0;
    private double mMinimumInput = 0.0;

    private double mPrevError = 0.0;
    private double mTotalError = 0.0;
    private double mError = 100.0;
    private double mDeadband = 0.0;
    private double mSlot = 0;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private double desiredOutput = 0;

    private Clock mClock;

    private EPIDControlType mPIDType;
    private ProfileGains mGains;

    public enum EPIDControlType {
        VELOCITY,
        POSITION
    }

    public ILITEPIDController(EPIDControlType pPIDType, ProfileGains pGains, Clock pClock) {
        mPIDType = pPIDType;
        mGains = pGains;
        mClock = pClock;
    }

    public void setInputRange(double pMinInput, double pMaxInput) {
        mMinimumInput = pMinInput;
        mMaximumInput = pMaxInput;
    }

    public void setOutputRange(double pMinOutput, double pMaxOutput) {
        mMinimumOutput = pMinOutput;
        mMaximumOutput = pMaxOutput;
    }

    public double calculate(double measurement, double setpoint) {
        mCurrentTime = mClock.getCurrentTimeInMillis();
        mError = setpoint - measurement;

        double dT = mCurrentTime - mPreviousTime;
        double dX = mError - mPrevError;

        if ( ( mError * mProfileGains.P < mMaximumOutput ) && ( mError * mProfileGains.P > mMinimumOutput ) ) {
            mTotalError += mError * mClock.dt();
        } else {
            mTotalError = 0;
        }

        double proportionalError = Math.abs( mError ) < mDeadband ? 0 : mError;

        double proportion = mGains.P * proportionalError;
        double integral = mGains.I * mTotalError;
        double derivative = mGains.D * dX/dT;
        double feedforward = mGains.F * setpoint;

        switch(mPIDType) {
            case VELOCITY:
                desiredOutput = proportion + integral + derivative + feedforward;
            case POSITION:
                desiredOutput = proportion + integral + derivative;
        }

        desiredOutput = Utils.clamp( desiredOutput, mMinimumOutput, mMaximumOutput );

        mPreviousTime = mCurrentTime;
        mPrevError = mError;

        return desiredOutput;
    }
}
