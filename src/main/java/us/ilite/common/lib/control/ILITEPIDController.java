package us.ilite.common.lib.control;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.lib.util.Utils;
import us.ilite.robot.hardware.Clock;

import static java.lang.Math.*;

public class ILITEPIDController implements Sendable {

    private EPIDControlType mPIDType;
    private final ProfileGains mGains;
    private final Clock mClock;

    // Configurations
    private double mMaximumOutput;
    private double mMinimumOutput;
    private double mMaximumInput = 0.0;
    private double mMinimumInput = 0.0;

    // Gains
    private final double MAX_ACCEL;
    private double mP;
    private double mI;
    private double mD;
    private final double kF;

    // Calculations
    private double mPreviousTime;
    private double mPrevError = 0.0;
    private double mTotalError = 0.0;
    private double desiredOutput = 0;

    /**
     * Determines whether the output is in velocity mode or position mode
     */
    public enum EPIDControlType {
        VELOCITY,
        POSITION
    }

    public ILITEPIDController(EPIDControlType pPIDType, ProfileGains pGains, Clock pClock) {
        mPIDType = pPIDType;
        mGains = pGains;
        mClock = pClock;

        mP = pGains.P;
        mI = pGains.I;
        mD = pGains.D;
        kF = pGains.F;

        MAX_ACCEL = mGains.MAX_ACCEL;
        mMaximumOutput = mGains.MAX_VELOCITY;
        mMinimumOutput = -mGains.MAX_VELOCITY;
    }

    /**
     * Sets the minimum and maximum allowed input
     * @param pMinInput the minimum input value accepted
     * @param pMaxInput the maximum input value accepted
     */
    public void setInputRange(double pMinInput, double pMaxInput) {
        mMinimumInput = pMinInput;
        mMaximumInput = pMaxInput;
    }

    /**
     * Sets the minimum and maximum allowed output
     * @param pMinOutput the minimum output value accepted
     * @param pMaxOutput the maximum output value accepted
     */
    public void setOutputRange(double pMinOutput, double pMaxOutput) {
        mMinimumOutput = pMinOutput;
        mMaximumOutput = pMaxOutput;
    }

    /**
     * Calculates the desired output within the output boundaries
     * @param measurement the current velocity or position
     * @param setpoint the desired velocity or position
     */
    public double calculate(double measurement, double setpoint) {
        setpoint = min(max(setpoint, mMinimumInput), mMaximumInput);
        double mError = setpoint - measurement;

        double mCurrentTime = mClock.getCurrentTimeInMillis();
        double dT = mCurrentTime - mPreviousTime;
        double dX = Utils.clamp(mError - mPrevError, MAX_ACCEL);

        if (max(min(mMaximumOutput, mError * mP), mMinimumOutput) == mError * mP) {
            mTotalError += mError * mClock.dt();
        } else {
            mTotalError = 0;
        }

        double proportion = -mP * mError;
        double integral = -mI * mTotalError;
        double derivative = -mD * dX/dT;
        double feedforward = kF * setpoint;

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

    /**
     * Calculates the desired output within the output boundaries
     * @param measurement the current velocity or position
     * @param setpoint the desired velocity or position
     * @param pPIDType the desired output type
     */
    public double calculate(double measurement, double setpoint, EPIDControlType pPIDType) {
        mPIDType = pPIDType;
        setpoint = min(max(setpoint, mMinimumInput), mMaximumInput);
        double mError = setpoint - measurement;

        double mCurrentTime = mClock.getCurrentTimeInMillis();
        double dT = mCurrentTime - mPreviousTime;
        double dX = Utils.clamp(mError - mPrevError, MAX_ACCEL);

        if (max(min(mMaximumOutput, mError * mP), mMinimumOutput) == mError * mP) {
            mTotalError += mError * mClock.dt();
        } else {
            mTotalError = 0;
        }

        double proportion = mP * mError;
        double integral = mI * mTotalError;
        double derivative = mD * dX/dT;
        double feedforward = kF * setpoint;

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

    private void setP(double pPID) {
        mP = pPID;
        SmartDashboard.putNumber("P", mP);
    }
    private void setI(double pI) {
        mI = pI;
        SmartDashboard.putNumber("I", mI);
    }

    private void setD(double pD) {
        mD = pD;
        SmartDashboard.putNumber("D", mD);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", ()-> mP, this::setP);
        builder.addDoubleProperty("i", ()-> mI, this::setI);
        builder.addDoubleProperty("d", ()-> mD, this::setD);
        builder.addDoubleProperty("setpoint", ()->0d, this::setSetPoint);
    }
    private void setSetPoint(double pSetpoint) {
        //deliberate blank
    }
}
