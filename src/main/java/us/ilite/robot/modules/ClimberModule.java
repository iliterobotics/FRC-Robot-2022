package us.ilite.robot.modules;

import com.revrobotics.*;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.hardware.SparkMaxFactory;

public class ClimberModule extends Module{

    private CANSparkMax mSparkMaxOne;
    private RelativeEncoder mEncdoerSparkMaxOne;

    public ClimberModule() {
        mSparkMaxOne = SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncdoerSparkMaxOne = mSparkMaxOne.getEncoder();
    }
    @Override
    public void readInputs() {
        db.hanger.set(EHangerModuleData.L_VEL_rpm, mEncdoerSparkMaxOne.getVelocity());
    }

    @Override
    public void setOutputs() {
        mSparkMaxOne.set(db.hanger.get(EHangerModuleData.SET_pct));
    }
}
