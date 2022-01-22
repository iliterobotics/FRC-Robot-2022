package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.hardware.SparkMaxFactory;

import java.io.IOException;

public class ClimberModule extends Module{

    private CANSparkMax mSparkMaxOne;
    private RelativeEncoder mEncoderSparkMaxOne;
    private PIDController mHangerPid;
    private ProfileGains kHangerProfile;
    private double maxVelocity;
    private double minVelocity;
    private final RobotCodex<EHangerModuleData>mHangerModule;

    public ClimberModule() {
        mSparkMaxOne = SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoderSparkMaxOne = mSparkMaxOne.getEncoder();
        kHangerProfile = new ProfileGains().p(.001).i(0).d(0);
        //mHangerPid = new PIDController(kHangerProfile, -maxVelocity, minVelocity, clock.dt());
        mHangerModule = db.hanger;
    }

    ClimberModule(CANSparkMax pSparkMaxOne, RobotCodex<EHangerModuleData>hangerModule, RelativeEncoder pEncoderSparkMaxOne) {
        mSparkMaxOne = pSparkMaxOne;
        mHangerModule = hangerModule;
        mEncoderSparkMaxOne = pEncoderSparkMaxOne;

    }
    @Override
    public void readInputs() {
        try {
            mHangerModule.set(EHangerModuleData.L_VEL_rpm, mEncoderSparkMaxOne.getVelocity());
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void setOutputs() {
        mSparkMaxOne.set(mHangerModule.get(EHangerModuleData.SET_pct));
        //double desiredVelocity = mHangerPid.
    }
}
