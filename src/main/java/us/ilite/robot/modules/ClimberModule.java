package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.io.IOException;

public class ClimberModule extends Module{
    private TalonFX mLeftFalcon;
    private TalonFX mRightFalcon;
    private RelativeEncoder mEncoderSparkMaxOne;
    private ILITEPIDController mPidController;
    private ProfileGains kHangerProfile;
    private Clock mClock = new Clock();
    private final RobotCodex<EHangerModuleData>mHangerModule;

    public ClimberModule() {
        kHangerProfile = new ProfileGains().p(.0001).i(0).d(0);
        mPidController = new ILITEPIDController(ILITEPIDController.EPIDControlType.VELOCITY, kHangerProfile, mClock);
        mPidController.setOutputRange(-6380, 6380);
        mPidController.setInputRange(-100, 100);
        mLeftFalcon = new TalonFX(16);
        mRightFalcon = new TalonFX(17);
        mHangerModule = db.hanger;
    }

    ClimberModule(TalonFX pFalconOne, RobotCodex<EHangerModuleData>hangerModule, RelativeEncoder pEncoderSparkMaxOne) {
        mLeftFalcon = pFalconOne;
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
        db.hanger.set(EHangerModuleData.L_VEL_rpm, mLeftFalcon.getSelectedSensorPosition());
        db.hanger.set(EHangerModuleData.R_VEL_rpm, mRightFalcon.getSelectedSensorPosition());
    }

    @Override
    public void setOutputs() {
        mLeftFalcon.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.L_VEL_rpm), db.hanger.get(EHangerModuleData.L_DESIRED_VEL)));
        mRightFalcon.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.R_VEL_rpm), db.hanger.get(EHangerModuleData.R_DESIRED_VEL)));
    }
}
