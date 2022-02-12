package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.io.IOException;

public class ClimberModule extends Module{
    private TalonFX mLeftFalcon;
    private TalonFX mRightFalcon;
    private RelativeEncoder mEncoderSparkMaxOne;
    private ILITEPIDController mPidController;
    private DigitalBeamSensor mBeakBreak;
    private ProfileGains kHangerProfile;
    private Clock mClock = new Clock();
    private final RobotCodex<EHangerModuleData>mHangerModule;
    private double kGearboxRatio = 268.8;

    public ClimberModule() {
        kHangerProfile = new ProfileGains().p(.0001).i(0).d(0);
        mPidController = new ILITEPIDController(ILITEPIDController.EPIDControlType.VELOCITY, kHangerProfile, mClock);
        mPidController.setOutputRange(-6380, 6380);
        mBeakBreak = new DigitalBeamSensor(10);
        mPidController.setInputRange(-100, 100);
        mLeftFalcon = new TalonFX(16);
        mRightFalcon = new TalonFX(17);

        mHangerModule = db.hanger;
        mLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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
        db.hanger.set(EHangerModuleData.L_VEL_rpm, mLeftFalcon.getSelectedSensorVelocity());
        db.hanger.set(EHangerModuleData.R_VEL_rpm, mRightFalcon.getSelectedSensorVelocity());
        //db.hanger.set(EHangerModuleData.CURRENT_ARM_ANGLE, );
    }

    @Override
    public void setOutputs() {
        Enums.EHangerMode mode = db.hanger.get(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.class);
        if (mode == null) {
            mode = Enums.EHangerMode.DEFAULT;
        }

        switch(mode) {
            case VELOCITY:
                mLeftFalcon.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.L_VEL_rpm), db.hanger.get(EHangerModuleData.L_DESIRED_VEL)));
                mRightFalcon.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.R_VEL_rpm), db.hanger.get(EHangerModuleData.R_DESIRED_VEL)));
                break;
            case DEFAULT:
                mLeftFalcon.set(ControlMode.Velocity, 0);
                mRightFalcon.set(ControlMode.Velocity, 0);
                break;
            case POSITION:
                mLeftFalcon.set(ControlMode.Position, mPidController.calculate(db.hanger.get(EHangerModuleData.L_VEL_rpm), db.hanger.get(EHangerModuleData.L_DESIRED_VEL)));
                mRightFalcon.set(ControlMode.Position, mPidController.calculate(db.hanger.get(EHangerModuleData.R_VEL_rpm), db.hanger.get(EHangerModuleData.R_DESIRED_VEL)));
                break;
        }
    }
}
