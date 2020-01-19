package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.robot.hardware.SparkMaxFactory;

public class HangerModule extends Module {

    private CANSparkMax mHangerNeoOne;
    private CANSparkMax mHangerNeoTwo;
    private Data mData;

    public HangerModule(Data pData){
        this.mData = pData;
        mHangerNeoOne = SparkMaxFactory.createDefaultSparkMax(Settings.kHangerNeoID1 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mHangerNeoTwo= SparkMaxFactory.createDefaultSparkMax(Settings.kHangerNeoID2 , CANSparkMaxLowLevel.MotorType.kBrushless);

    }
    public enum EHangerState {
        HANGING(1.0),
        NOT_HANGING(0.0);

        private double power;
        EHangerState(double power){
            this.power = power;
        }

    }

    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {

    }
}
