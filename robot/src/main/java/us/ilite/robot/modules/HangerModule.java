package us.ilite.robot.modules;

import com.revrobotics.*;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class HangerModule extends Module {

    private CANSparkMax mHangerNeoMaster;
    private CANSparkMax mHangerNeoFollower;

    private EHangerState mHangerState;
//    private CANPIDController mHangerPID;

    private CANEncoder mHangerEncoderOne;
    private CANEncoder mHangerEncoderTwo;

    //PID Constants, to be used if needed
//    private static final int UP_PID_SLOT_ID = 1;
//    public static double P = 5.0e-4;
//    public static double I = 0.0;
//    public static double D = 0.0;
//    public static double F = 0.000391419;
//    public static double kMaxElevatorVelocity = 3700;
//    public static double kMinElevatorVelocity = 0;
//    public static double kMaxElevatorUpAcceleration = 4000 * 1.5;
//    public static double kMaxElevatorDownAcceleration = 4000 * 1.5;

    private  double kHangerWarnCurrentLimitThreshold = 30;

    public HangerModule(){

        mHangerNeoMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID1 ,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mHangerNeoFollower = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID2 , CANSparkMaxLowLevel.MotorType.kBrushless);


//        mHangerPID = new CANPIDController(mHangerNeoMaster);
//        mHangerPID.setP(P, UP_PID_SLOT_ID);
//        mHangerPID.setI(I, UP_PID_SLOT_ID);
//        mHangerPID.setD(D, UP_PID_SLOT_ID);
//        mHangerPID.setSmartMotionMaxAccel(kMaxElevatorUpAcceleration, UP_PID_SLOT_ID);
//        mHangerPID.setSmartMotionMaxVelocity(kMaxElevatorVelocity, UP_PID_SLOT_ID);

        mHangerNeoMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mHangerNeoFollower.setInverted(false);
        mHangerNeoMaster.setInverted(true);


        mHangerNeoMaster.burnFlash();
        mHangerNeoFollower.burnFlash();

        mHangerEncoderOne = mHangerNeoMaster.getEncoder();
        mHangerEncoderTwo = mHangerNeoFollower.getEncoder();

        zeroTheEncoders();
    }
    public enum EHangerState {
        HANGING(1.0),
        BRAKE (0.0),
        REVERSE(-1.0);

        private double position;
        EHangerState(double position){
            this.position = position;
        }
        private double getPower(){
            return this.position;
        }

    }

    @Override
    public void readInputs(double pNow) {
//        db.hanger.set(EHangerModuleData.CURRENT_HANGER_VELOCITY , mHangerEncoderOne.getVelocity());
//        db.hanger.set(EHangerModuleData.CURRENT_HANGER_VELOCITY , mHangerEncoderTwo.getVelocity());

//        db.hanger.set(EHangerModuleData.CURRENT_POSITION, mHangerEncoderOne.getPosition());
//        db.hanger.set(EHangerModuleData.CURRENT_POSITION, mHangerEncoderTwo.getPosition());

        db.hanger.set(EHangerModuleData.OUTPUT_CURRENT, mHangerNeoMaster.getOutputCurrent());
        db.hanger.set(EHangerModuleData.OUTPUT_CURRENT, mHangerNeoFollower.getOutputCurrent());
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow){


    }

    @Override
    public void setOutputs(double pNow) {
        mHangerNeoFollower.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_PCT));
        mHangerNeoMaster.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_PCT));
    }

    public EHangerState returnHangerState() {
        return this.mHangerState;
    }
    public void putDesiredHangerState(EHangerState desiredState){
        mHangerState = desiredState;
    }
    public boolean isCurrentLimiting(){
        return mHangerNeoMaster.getOutputCurrent() >= kHangerWarnCurrentLimitThreshold;
    }

    public void zeroTheEncoders(){
        mHangerEncoderOne.setPosition(0);
    }

//    public boolean isAtPosition() {
//        return mCurrentState == EElevatorState.SET_POSITION && (Math.abs(pPosition.getEncoderRotations() - mData.elevator.get(EElevator.CURRENT_ENCODER_TICKS)) <= SystemSettings.kElevatorAllowableError);
//    }

}
