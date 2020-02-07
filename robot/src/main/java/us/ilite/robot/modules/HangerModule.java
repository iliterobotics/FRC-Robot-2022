package us.ilite.robot.modules;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class HangerModule extends Module {

    private CANSparkMax mHangerNeoOne;
    private CANSparkMax mHangerNeoTwo;

    private EHangerState mHangerState;
    private CANPIDController mHangerPID;

    private CANEncoder mHangerEncoderOne;
//    private CANEncoder mHangerEncoderTwo; O

    //PID Constants, to be used if needed
    private double k_P = 0.5; //Change later, tune this
    private double k_I = 0.5; //Change later, tune this
    private double k_D = 0.5; //Change later, tune this
    private double k_FF = 0.5; //Change later, tune this

    private  double kHangerWarnCurrentLimitThreshold = 30;

    public HangerModule(){

        mHangerNeoOne = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID1 ,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mHangerNeoTwo = SparkMaxFactory.createFollowerSparkMax(Settings.Hardware.CAN.kHangerNeoID2 , mHangerNeoOne,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mHangerPID = new CANPIDController(mHangerNeoOne);

        mHangerNeoTwo.follow(mHangerNeoOne , true);

        mHangerPID.setP(k_P);
        mHangerPID.setP(k_I);
        mHangerPID.setP(k_D);
        mHangerPID.setP(k_FF);
        

        mHangerNeoOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mHangerNeoOne.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mHangerNeoOne.burnFlash();
        mHangerNeoTwo.burnFlash();

        mHangerEncoderOne = mHangerNeoOne.getEncoder();

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
//        Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , (double) returnHangerState().ordinal() );
//        Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , (double) returnHangerState().ordinal() );

        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_HANGER_POSITION , mHangerNeoOne.getOutputCurrent());
    }
    
    public void modeInit(){


    }

    @Override
    public void setOutputs(double pNow) {
        switch (mHangerState){
            case HANGING:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION , 1.0);
            case BRAKE:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION , 0.0);
        }
        mHangerNeoOne.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_HANGER_POSITION));

    }



    public EHangerState returnHangerState() {
        return this.mHangerState;
    }
    public void putDesiredHangerState(EHangerState desiredState){
        mHangerState = desiredState;
    }
    public boolean isCurrentLimiting(){
        return mHangerNeoOne.getOutputCurrent() >= kHangerWarnCurrentLimitThreshold;
        //Will change the PDP location of the current
    }

    public void zeroTheEncoders(){
        mHangerEncoderOne.setPosition(0);
        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_ENCODER_TICKS, 0.0);
    }

//    public boolean isAtPosition() {
//        return mCurrentState == EElevatorState.SET_POSITION && (Math.abs(pPosition.getEncoderRotations() - mData.elevator.get(EElevator.CURRENT_ENCODER_TICKS)) <= SystemSettings.kElevatorAllowableError);
//    }

}
