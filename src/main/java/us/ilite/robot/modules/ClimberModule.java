package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.*;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.DigitalBeamSensor;

public class ClimberModule extends Module{
    private TalonFX mCLL0;
    private TalonFX mCLMR0;
    private DigitalBeamSensor mCLBeamCheck;

    private ILITEPIDController mPidController;
    private ProfileGains kHangerProfile;
    private Clock mClock = new Clock();
    private final RobotCodex<EHangerModuleData>mHangerModule;
    private double kGearboxRatio = 268.8;

    //verify these:
    private double kWheelDiameterInches = 5.875;
    private double kWheelCircumferenceFeet = kWheelDiameterInches * Math.PI/12.0;

    public ClimberModule() {
        kHangerProfile = new ProfileGains().p(.00001).i(0).d(0);
        mPidController = new ILITEPIDController(ILITEPIDController.EPIDControlType.VELOCITY, kHangerProfile, mClock);
        //verify these:
        mPidController.setOutputRange(-6380, 6380);
        mPidController.setInputRange(-100, 100);

        //CHANGE THESE IDS:
        mCLBeamCheck = new DigitalBeamSensor(-1);
        mCLL0 = new TalonFX(-1);
        mCLMR0 = new TalonFX(-1);

        mHangerModule = db.hanger;
        mCLL0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    ClimberModule(TalonFX pFalconOne, RobotCodex<EHangerModuleData>hangerModule, RelativeEncoder pEncoderSparkMaxOne) {
        mCLL0 = pFalconOne;
        mHangerModule = hangerModule;
    }

    @Override
    public void readInputs() {
        try {
            mHangerModule.set(EHangerModuleData.L_VEL_rpm, mCLMR0.getSelectedSensorVelocity());
        } catch(Exception e) {
            e.printStackTrace();
        }
        //convert from raw sensor velocity inputs to feet per second
        double leftRawSensorVelocity = mCLL0.getSelectedSensorVelocity();
        double leftRotationsPerSecond = (leftRawSensorVelocity / 2048) / 1000;
        double leftFeetPerSecond = leftRotationsPerSecond / kWheelCircumferenceFeet;

        double rightRawSensorVelocity = mCLMR0.getSelectedSensorVelocity();
        double rightRotationsPerSecond = (rightRawSensorVelocity / 2048) / 1000;
        double rightFeetPerSecond = rightRotationsPerSecond / kWheelCircumferenceFeet;

        //convert from raw sensor position inputs to feet
        double leftRawSensorPosition = mCLL0.getSelectedSensorPosition();
        double leftRotations = (leftRawSensorPosition / 2048);
        double leftFeet = leftRotations / kWheelCircumferenceFeet;

        double rightRawSensorPosition = mCLMR0.getSelectedSensorPosition();
        double rightRotations = (rightRawSensorPosition / 2048);
        double rightFeet = rightRotations / kWheelCircumferenceFeet;

        db.hanger.set(EHangerModuleData.L_VEL_rpm,leftFeetPerSecond);
        db.hanger.set(EHangerModuleData.R_VEL_rpm, rightFeetPerSecond);
        db.hanger.set(EHangerModuleData.L_POSITION_rot, leftFeet);
        db.hanger.set(EHangerModuleData.R_POSITION_rot, rightFeet);
        if (mCLBeamCheck.isBroken()) {
            db.hanger.set(EHangerModuleData.BAR_BEAM, 1);
        }
        else {
            db.hanger.set(EHangerModuleData.BAR_BEAM, 0);
        }
    }

    @Override
    public void setOutputs() {
        Enums.EHangerMode mode = db.hanger.get(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.class);
        if (mode == null) {
            mode = Enums.EHangerMode.DEFAULT;
        }

        switch(mode) {
            case VELOCITY:
                mCLL0.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.L_VEL_rpm), db.hanger.get(EHangerModuleData.L_DESIRED_VEL_rpm)));
                mCLMR0.set(ControlMode.Velocity, mPidController.calculate(db.hanger.get(EHangerModuleData.R_VEL_rpm), db.hanger.get(EHangerModuleData.R_DESIRED_VEL_rpm)));
                break;
            case DEFAULT:
                mCLL0.set(ControlMode.Velocity, 0);
                mCLMR0.set(ControlMode.Velocity, 0);
                break;
            case POSITION:
                mCLL0.set(ControlMode.Position, mPidController.calculate(db.hanger.get(EHangerModuleData.L_POSITION_rot), db.hanger.get(EHangerModuleData.L_DESIRED_POSITION_rot)));
                mCLMR0.set(ControlMode.Position, mPidController.calculate(db.hanger.get(EHangerModuleData.R_POSITION_rot), db.hanger.get(EHangerModuleData.R_DESIRED_POSITION_rot)));
                break;
        }
    }
}
