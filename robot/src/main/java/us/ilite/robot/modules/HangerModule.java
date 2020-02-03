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
    private CANPIDController mHangerPID2;

    private CANEncoder mHangerEncoderOne;
    private CANEncoder mHangerEncoderTwo;

    //PID Constants, to be used if needed
    private double k_P = 0.5; //Change later, tune this
    private double k_I = 0.5; //Change later, tune this
    private double k_D = 0.5; //Change later, tune this
    private double k_FF = 0.5; //Change later, tune this

    private  double kHangerWarnCurrentLimitThreshold = 30;

    public HangerModule(){

        mHangerNeoOne = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID1 ,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mHangerNeoTwo = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID2 ,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mHangerPID = new CANPIDController(mHangerNeoOne);
        mHangerPID2 = new CANPIDController(mHangerNeoTwo);

        mHangerNeoTwo.follow(mHangerNeoOne , true);

        mHangerPID.setP(k_P);
        mHangerPID.setP(k_I);
        mHangerPID.setP(k_D);
        mHangerPID.setP(k_FF);

        mHangerPID2.setP(k_P);
        mHangerPID2.setP(k_I);
        mHangerPID2.setP(k_D);
        mHangerPID2.setP(k_FF);


        mHangerNeoOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mHangerNeoOne.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mHangerNeoOne.burnFlash();
        mHangerNeoTwo.burnFlash();

        mHangerEncoderOne = mHangerNeoOne.getEncoder();
        mHangerEncoderTwo = mHangerNeoTwo.getEncoder();

        zeroTheEncoders();

    }
    public enum EHangerState {
        HANGING(1.0),
        NOT_HANGING(0.0),
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

        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_HANGER_POSITION1 , mHangerNeoOne.getOutputCurrent());
        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_HANGER_POSITION2 , mHangerNeoTwo.getOutputCurrent());
    }
    
    public void modeInit(){


    }

    @Override
    public void setOutputs(double pNow) {
        mHangerNeoOne.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_HANGER_POSITION1));
        mHangerNeoTwo.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_HANGER_POSITION2));

        switch (mHangerState){
            case HANGING:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION1 , 1.0);
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION2 , 1.0);
            case NOT_HANGING:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION1 , 0.0);
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POSITION2 , 0.0);
        }

    }



    public EHangerState returnHangerState() {
        return this.mHangerState;
    }
    public void putDesiredHangerState(EHangerState desiredState){
        mHangerState = desiredState;
    }
    public boolean isCurrentLimiting(){
        return Robot.DATA.pdp.get(EPowerDistPanel.CURRENT9) > kHangerWarnCurrentLimitThreshold;
    }

    public void zeroTheEncoders(){
        mHangerEncoderOne.setPosition(0);
        mHangerEncoderTwo.setPosition(0);
        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_ENCODER_TICKS, 0.0);
    }

//    public boolean isAtPosition() {
//        return mCurrentState == EElevatorState.SET_POSITION && (Math.abs(pPosition.getEncoderRotations() - mData.elevator.get(EElevator.CURRENT_ENCODER_TICKS)) <= SystemSettings.kElevatorAllowableError);
//    }

}
