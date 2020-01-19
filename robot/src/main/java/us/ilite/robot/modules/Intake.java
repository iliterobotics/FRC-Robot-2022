package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

public class Intake extends Module {

    // Intake Motors
    private CANSparkMax mCANMotor;
    private TalonSRX mTalonOne;
    private TalonSRX mTalonTwo;
    private TalonSRX mTalonThree;

    //Beam Breakers
    private DigitalBeamSensor mBeamBreaker1;
    private DigitalBeamSensor mBeamBreaker2;
    private DigitalBeamSensor mBeamBreaker3;

    private Data mData;

    //Intake state
    private EIntakeState mIntakeState;

    private ILog mLog = Logger.createLog(this.getClass());


    public enum EIntakeState{
        INTAKE,
        STOP,
        REVERSE;
    }

    public Intake( Data pData ) {
        this.mData = pData;

        mCANMotor = SparkMaxFactory.createDefaultSparkMax( Settings.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mTalonOne = TalonSRXFactory.createDefaultTalon( Settings.kTalonOneID );
        mTalonTwo = TalonSRXFactory.createDefaultTalon( Settings.kTalonTwoID);
        mTalonThree = TalonSRXFactory.createDefaultTalon( Settings.kTalonThreeID);

        mBeamBreaker1 = new DigitalBeamSensor( Settings.kBeamChannel1);
        mBeamBreaker2 = new DigitalBeamSensor( Settings.kBeamChannel2);
        mBeamBreaker3 = new DigitalBeamSensor( Settings.kBeamChannel3);

    }

    @Override
    public void modeInit(double pNow) {

    }

    @Override
    public void periodicInput(double pNow) {

    }

    @Override
    public void update(double pNow) {
        mLog.info("------------------INTAKE UPDATE HAS BEGUN");
        switch (mIntakeState) {
            case INTAKE:
                mCANMotor.set(1.0);
                if ( mBeamBreaker1.isBroken() ){
                    mTalonOne.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);
                }
                if ( mBeamBreaker2.isBroken() ){
                    mTalonTwo.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);
                }
                if ( mBeamBreaker3.isBroken() ){
                    mTalonThree.set(ControlMode.PercentOutput, Settings.kIntakeTalonPower);
                }
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
        mCANMotor.set(0.0);
        mTalonOne.set(ControlMode.PercentOutput, 0d);
        mTalonTwo.set(ControlMode.PercentOutput, 0d);
        mTalonThree.set(ControlMode.PercentOutput, 0d);
    }
    public void setDesiredIntakeState(EIntakeState pDesiredState){
        mIntakeState = pDesiredState;
    }
    public EIntakeState returnIntakeState(){
        return this.mIntakeState;
    }

}
