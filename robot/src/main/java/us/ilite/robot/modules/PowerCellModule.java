package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.sensor.EPowerDistPanel;
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

    //Arm
    private CANSparkMax mArmMotor;

    //Beam Breakers
//    private DigitalBeamSensor mBeamBreaker1;
//    private DigitalBeamSensor mBeamBreaker2;
//    private DigitalBeamSensor mBeamBreaker3;

    //Array of beam-breakers (Used when indexing PowerCells)
    //private DigitalBeamSensor[] mDigitalBeamSensors;

    public static double kIntakeTalonPower = 1d;
    public static double kForStopTalon = 0d;
    public static double kIntakePower = 1.0;
    public static double kForStop = 1.0;
    public static double kPowerCellP;
    public static double kPowerCellI;
    public static double kPowerCellD;
    public static double kPowerCellF;
    public static double kWarnCurrentLimitThreshold = 30; //Tune this later
    public static double kArmPowerEngaged = 1.0;
    public static double kArmPowerDisengaged = 0.0;
    //Intake state
    private EIntakeState mIntakeState;

    //Arm State
    private EArmState mArmState;
    
    //For indexing
    private int mGoalBeamCountBroken = 0;
    private int mBeamCountBroken = 0;

//    private CANEncoder mNeoEncoder;
//    private CANPIDController mCanController;


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
    public enum EArmState {
        ENGAGED (1.0),
        DISENGAGED (0.0);


        private double power;

        EArmState (double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public PowerCellModule() {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mCANMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kCANIntakeID ,
                CANSparkMaxLowLevel.MotorType.kBrushless );
        mTalonOne = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonOneID );
        mTalonTwo = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonTwoID );
        mTalonThree = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonThreeID );

        mArmMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kArmNEOAdress ,
                CANSparkMaxLowLevel.MotorType.kBrushless);

//        mBeamBreaker1 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel1 );
//        mBeamBreaker2 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel2 );
//        mBeamBreaker3 = new DigitalBeamSensor( Settings.PowerCellModule.kBeamChannel3) ;

//        mDigitalBeamSensors = new DigitalBeamSensor[]{mBeamBreaker1, mBeamBreaker2, mBeamBreaker3};
//
//        mNeoEncoder = mCANMotor.getEncoder();
//        mCanController = mCANMotor.getPIDController();

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
//        mCanController.setP(Settings.PowerCellModule.kPowerCellP);
//        mCanController.setI(Settings.PowerCellModule.kPowerCellI);
//        mCanController.setD(Settings.PowerCellModule.kPowerCellD);
//        mCanController.setFF(Settings.PowerCellModule.kPowerCellF);
    }

    @Override
    public void readInputs(double pNow) {
//        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_TWO_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_TWO_POWER_PCT , mTalonTwo.getStatorCurrent());

//        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , mTalonTwo.getStatorCurrent());
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , (double) getIntakeState().ordinal());

//        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_CONVEYOR_POWER_PCT , mTalonOne.getStatorCurrent());

//        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , mTalonOne.getStatorCurrent());
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , (double) getIntakeState().ordinal());

//        Robot.DATA.powercell.set(EPowerCellData.CURRENT_SERIALIZER_POWER_PCT , (double) getIntakeState().ordinal());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_SERIALIZER_POWER_PCT, mCANMotor.getOutputCurrent());

//        Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT ,mCANMotor.getOutputCurrent() );
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , (double) getIntakeState().ordinal() );

        Robot.DATA.powercell.set(EPowerCellData.CURRENT_ARM_STATE , mArmMotor.getOutputCurrent() );
//        Robot.DATA.powercell.set(EPowerCellData.CURRENT_ARM_STATE , (double)getArmState().ordinal());

//        Robot.DATA.powercell.set(EPowerCellData.DESIRED_ARM_STATE , mArmMotor.getOutputCurrent() );
        Robot.DATA.powercell.set(EPowerCellData.DESIRED_ARM_STATE , (double)getArmState().ordinal());

//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_0 , readBeamBreakerState(mBeamBreaker1.isBroken()));
//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_1 , readBeamBreakerState(mBeamBreaker2.isBroken()));
//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_2 , readBeamBreakerState(mBeamBreaker3.isBroken()));

    }

    @Override
    public void setOutputs(double pNow) {
//        mCANMotor.set(EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
//        mTalonOne.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
//        mTalonTwo.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
//        mTalonThree.set(ControlMode.PercentOutput, EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());


//        mCANMotor.set(1.0);
        switch (mArmState) {
            case ENGAGED:
                mArmMotor.set( kArmPowerEngaged);
                break;
            case DISENGAGED:
                mArmMotor.set( -kArmPowerDisengaged);
                break;
        }

    }

    @Override
    public void shutdown(double pNow) {
        mIntakeState = EIntakeState.STOP;
    }

    public EIntakeState getIntakeState(){
        return this.mIntakeState;
    }

    public EArmState getArmState(){
        return this.mArmState;
    }

//    public void indexPowerCells() {
//        mLog.info("----------------INDEXING OF POWERCELLS HAS BEGUN------------------");
//        mBeamCountBroken = (int) List.of(mDigitalBeamSensors).stream().map(DigitalBeamSensor::isBroken).filter(e -> e).count();
//        if ( mBeamCountBroken < mGoalBeamCountBroken) {
//            setDesiredIntakeState(EIntakeState.INTAKE);
//        } else {
//            setDesiredIntakeState(EIntakeState.STOP);
//        }
//        mGoalBeamCountBroken = mBeamCountBroken + 1;
//    }
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

    public boolean isCurrentLimiting(){
        return Robot.DATA.pdp.get(EPowerDistPanel.CURRENT5) > kWarnCurrentLimitThreshold;
    }


}
