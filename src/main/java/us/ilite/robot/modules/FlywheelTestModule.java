package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelTestModule extends Module {


    private CANSparkMax mFlywheel;
    private RelativeEncoder mFlywheelEncoder;

    public FlywheelTestModule()
    {
        mFlywheel= SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mFlywheelEncoder = mFlywheel.getEncoder();
    }



    @Override
    public void readInputs() {
        db.flywheel.set(EShooterSystemData.BALL_VELOCITY_ft_s,mFlywheelEncoder.getVelocity());


    }

    @Override
    public void setOutputs() {
        mFlywheel.set(db.flywheel.get(EShooterSystemData.SET_BALL_VELOCITY_ft_s));


    }
}
