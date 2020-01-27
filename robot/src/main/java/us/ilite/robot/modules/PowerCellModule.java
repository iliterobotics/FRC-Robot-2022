package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.util.List;

public class PowerCellModule extends Module {

    // Intake Motors
    private CANSparkMax mCANMotor;
    private TalonSRX mTalonOne;
    private TalonSRX mTalonTwo;
    private TalonSRX mTalonThree;

    //Beam Breakers
    private DigitalBeamSensor mBeamBreaker1;
    private DigitalBeamSensor mBeamBreaker2;
    private DigitalBeamSensor mBeamBreaker3;

    private DigitalBeamSensor[] mDigitalBeamSensors;

    //Intake state
    private EIntakeState mIntakeState;
    
    //For indexing
    private int mGoalBeamCountBroken = 0;
    private int mBeamCountBroken = 0;

    private ILog mLog = Logger.createLog(this.getClass());

    public enum EIntakeState {
        INTAKE (1.0),
        STOP (0.0),
        REVERSE (-1.0);

        private double power;

        EIntakeState (double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public PowerCellModule() {

        mCANMotor = SparkMaxFactory.createDefaultSparkMax( Settings.PowerCellModule.kCANIntakeID ,
                CANSparkMaxLowLevel.MotorType.kBrushless );
        mTalonOne = TalonSRXFactory.createDefaultTalon( Settings.PowerCellModule.kTalonOneID );
        mTalonTwo = TalonSRXFactory.createDefaultTalon( Settings.PowerCellModule.kTalonTwoID );
        mTalonThree = TalonSRXFactory.createDefaultTalon( Settings.PowerCellModule.kTalonThreeID );

        mBeamBreaker1 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel1 );
        mBeamBreaker2 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel2 );
        mBeamBreaker3 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel3) ;

        mDigitalBeamSensors = new DigitalBeamSensor[]{mBeamBreaker1, mBeamBreaker2, mBeamBreaker3};

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_TWO_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_TWO_POWER_PCT , mTalonTwo.getOutputCurrent());

        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , mTalonTwo.getOutputCurrent());
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , (double) getIntakeState().ordinal());

        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_POWER_PCT , mTalonOne.getOutputCurrent());

        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , mTalonOne.getOutputCurrent());
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , (double) getIntakeState().ordinal());

        Robot.DATA.powercell.set(EPowerCellData.CURRENT_SERIALIZER_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_SERIALIZER_POWER_PCT, mCANMotor.getOutputCurrent());

        Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT ,mCANMotor.getOutputCurrent() );
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , (double) getIntakeState().ordinal() );

        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_0 , readBeamBreakerState(mBeamBreaker1.isBroken()));
        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_1 , readBeamBreakerState(mBeamBreaker2.isBroken()));
        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_2 , readBeamBreakerState(mBeamBreaker3.isBroken()));

    }

    @Override
    public void setOutputs(double pNow) {
        mCANMotor.set(EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonOne.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonTwo.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonThree.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());

        switch (mIntakeState) {
            case INTAKE:
                mCANMotor.set( Settings.PowerCellModule.kIntakeCANPower);
                mTalonOne.set(ControlMode.PercentOutput, Settings.PowerCellModule.kIntakeTalonPower);
                mTalonTwo.set(ControlMode.PercentOutput, Settings.PowerCellModule.kIntakeTalonPower);
                break;
            case REVERSE:
                mCANMotor.set( -Settings.PowerCellModule.kIntakeCANPower);
                mTalonOne.set(ControlMode.PercentOutput, Settings.PowerCellModule.kIntakeTalonPower);
                mTalonTwo.set(ControlMode.PercentOutput, Settings.PowerCellModule.kIntakeTalonPower);
                break;
            case STOP:
                mCANMotor.set(Settings.PowerCellModule.kForStopCAN);
                mTalonOne.set(ControlMode.PercentOutput, Settings.PowerCellModule.kForStopTalon);
                mTalonTwo.set(ControlMode.PercentOutput, Settings.PowerCellModule.kForStopTalon);
                break;
        }
    }

    @Override
    public void shutdown(double pNow) {
       Robot.DATA.powercell.set(EIntakeState.STOP.ordinal() , 0.0);
    }
    public void setDesiredIntakeState(EIntakeState pDesiredState){
        mIntakeState = pDesiredState;
    }

    public EIntakeState getIntakeState(){
        return this.mIntakeState;
    }
    
    public void indexPowecells() {
        mBeamCountBroken = (int) List.of(mDigitalBeamSensors).stream().map(DigitalBeamSensor::isBroken).filter(e -> e).count();
        if ( mBeamCountBroken < mGoalBeamCountBroken) {
            setDesiredIntakeState(EIntakeState.INTAKE);
        } else {
            setDesiredIntakeState(EIntakeState.STOP);
        }
        mGoalBeamCountBroken = mBeamCountBroken + 1;
    }
//    public void startIntaking() {
//        mCANMotor.set(1.0);
//        mTalonOne.set(ControlMode.PercentOutput, 1d);
//        mTalonTwo.set(ControlMode.PercentOutput, 1d);
//        mTalonThree.set(ControlMode.PercentOutput, 1d);
//    }
//    public void startIndexing() {
//        mCANMotor.set(1.0);
//        if ( mBeamBreaker1.isBroken() ){
//
//        }
//    }
    private double readBeamBreakerState(boolean isBroken){
        if ( isBroken ){
            return 1.0;
        }
        return 0.0;
    }


}
