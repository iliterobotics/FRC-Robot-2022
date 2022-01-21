package us.ilite.robot.modules;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.hardware.SparkMaxFactory;

public class ClimberModule extends Module{

    private CANSparkMax mSparkMaxOne;
    private RelativeEncoder mEncdoerSparkMaxOne;
    private PIDController mHangerPid;
    private ProfileGains kHangerProfile;
    private double maxVelocity;
    private double minVelocity;

    public ClimberModule() {
        mSparkMaxOne = SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncdoerSparkMaxOne = mSparkMaxOne.getEncoder();
        kHangerProfile = new ProfileGains().p(.001).i(0).d(0);
        mHangerPid = new PIDController(kHangerProfile, -maxVelocity, minVelocity, clock.dt());
    }
    @Override
    public void readInputs() {
        db.hanger.set(EHangerModuleData.L_VEL_rpm, mEncdoerSparkMaxOne.getVelocity());
    }

    @Override
    public void setOutputs() {
        mSparkMaxOne.set(db.hanger.get(EHangerModuleData.SET_pct));
        double desiredVelocity = mHangerPid.
    }
}
