package us.ilite.robot.modules;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class HangerModule extends Module {

    private CANSparkMax mHangerNeoOne;
    private CANSparkMax mHangerNeoTwo;

    private EHangerState mHangerState;
    private CANPIDController mHangerPID;
    private CANPIDController mHangerPID2;
    private double k_P = 0.5; //Change later
    private double k_I = 0.5; //Change later
    private double k_D = 0.5; //Change later


    public HangerModule(){

        mHangerNeoOne = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID1 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mHangerNeoTwo= SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID2 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mHangerPID = new CANPIDController(mHangerNeoOne);
        mHangerPID2 = new CANPIDController(mHangerNeoTwo);

    }
    public enum EHangerState {
        HANGING(1.0),
        NOT_HANGING(0.0);

        private double power;
        EHangerState(double power){
            this.power = power;
        }
        private double getPower(){
            return this.power;
        }

    }

    @Override
    public void readInputs(double pNow) {
//        Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , (double) returnHangerState().ordinal() );
//        Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , (double) returnHangerState().ordinal() );

        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_HANGER_POWER1 , mHangerNeoOne.getOutputCurrent());
        Robot.DATA.hanger.set(EHangerModuleData.CURRENT_HANGER_POWER2 , mHangerNeoTwo.getOutputCurrent());
    }
    
    public void modeInit(){
        mHangerPID.setP(k_P);
        mHangerPID.setP(k_I);
        mHangerPID.setP(k_D);

    }

    @Override
    public void setOutputs(double pNow) {
        mHangerNeoOne.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_HANGER_POWER1));
        mHangerNeoTwo.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_HANGER_POWER2));

    }



    public EHangerState returnHangerState() {
        return this.mHangerState;
    }
    public void putDesiredHangerState(EHangerState desiredState){
        mHangerState = desiredState;
    }
    public void isCurrentLimiting(){

    }

}
