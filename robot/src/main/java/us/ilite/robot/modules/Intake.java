package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.robot.hardware.DigitalBeamSensor;

public class Intake extends Module {

    // Intake Motors
    private CANSparkMax mIntakeMotor;
    private TalonSRX mTalonOne;
    private TalonSRX mTalonTwo;
    private TalonSRX mTalonThree;

    //Beam Breakers
    private DigitalBeamSensor mBeamBreaker1;
    private DigitalBeamSensor mBeamBreaker2;
    private DigitalBeamSensor mBeamBreaker3;

    private Data mData;

    private EIntakeState mIntakeState;

    private ILog mLog = Logger.createLog(this.getClass());


    public enum EIntakeState{
        INTAKE,
        STOP,
        REVERSE;
    }

    public Intake( Data pData ) {
        this.mData = pData;
    }

    @Override
    public void modeInit(double pNow) {

    }

    @Override
    public void periodicInput(double pNow) {

    }

    @Override
    public void update(double pNow) {
        switch (mIntakeState) {
            case INTAKE:
                mIntakeMotor.set(1.0);
                mTalonOne.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);
                mTalonTwo.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);
                mTalonThree.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);

                break;
            case REVERSE:
                mTalonOne.set(ControlMode.PercentOutput, -Settings.kIntakeTalonPower);
                mTalonTwo.set(ControlMode.PercentOutput, -Settings.kIntakeTalonPower);
                mTalonThree.set(ControlMode.PercentOutput, -Settings.kIntakeTalonPower);
                break;
            case STOP:
                mTalonOne.set(ControlMode.PercentOutput, 0d);
                mTalonTwo.set(ControlMode.PercentOutput, 0d);
                mTalonThree.set(ControlMode.PercentOutput, 0d);
                break;
        }

    }

    @Override
    public void shutdown(double pNow) {

    }
}
