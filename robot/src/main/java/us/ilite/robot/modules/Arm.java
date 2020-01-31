package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EArmData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class Arm extends Module {
    private CANSparkMax mNeoMotor;
    private Data mData;
    private EArmState mArmState;
    public Arm (Data pData){
        mNeoMotor = SparkMaxFactory.createDefaultSparkMax(Settings.Arm.kArmNeoAddress , CANSparkMaxLowLevel.MotorType.kBrushless);
        this.mData = pData;
    }
    public enum EArmState{
        ENGAGED (1.0),
        DISENGAGED (0.0);

        private double power;
        EArmState(double power){
            this.power = power;
        }
    }
    @Override
    public void readInputs(double pNow) {
        Robot.DATA.arm.set(EArmData.CURRENT_POSITION, (double) EArmData.CURRENT_POSITION.ordinal());
        Robot.DATA.arm.set(EArmData.TARGET_POSITION, (double) EArmData.TARGET_POSITION.ordinal());

    }

    @Override
    public void setOutputs(double pNow) {
        switch (mArmState){
            case ENGAGED:
                Robot.DATA.arm.get(EArmState.ENGAGED.ordinal());
            case DISENGAGED:
                Robot.DATA.arm.get(EArmState.DISENGAGED.ordinal());
        }


    }

    public void raiseArm(){
        Robot.DATA.arm.set(EArmData.TARGET_POSITION , (double) EArmState.ENGAGED.ordinal());

    }
}
