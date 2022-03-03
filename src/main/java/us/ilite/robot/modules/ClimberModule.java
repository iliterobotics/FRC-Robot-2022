package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.Enums;

public class ClimberModule extends Module{
    private TalonFX mCLL0;
    private TalonFX mCLMR0;
    private DoubleSolenoid mCLPNL;
    private DoubleSolenoid mCLPNR;

    public ClimberModule() {
        //change these ids:
        mCLMR0 = new TalonFX(11);
        mCLL0 = new TalonFX(12);
        mCLMR0.setNeutralMode(NeutralMode.Brake);
        mCLL0.setNeutralMode(NeutralMode.Brake);

        mCLPNL = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
        mCLPNR = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);

        mCLL0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void readInputs() {
        //convert from raw sensor velocity inputs to rpm
        double leftRawSensorVelocity = mCLL0.getSelectedSensorVelocity();
        double leftRotationsPerSecond = (leftRawSensorVelocity / 2048) / 1000;
        double leftRotationsPerMinute = leftRotationsPerSecond * 60;

        double rightRawSensorVelocity = mCLMR0.getSelectedSensorVelocity();
        double rightRotationsPerSecond = (rightRawSensorVelocity / 2048) / 1000;
        double rightRotationsPerMinute = rightRotationsPerSecond * 60;

        //convert from raw sensor position inputs rpm
        double leftRawSensorPosition = mCLL0.getSelectedSensorPosition();
        double leftRotations = (leftRawSensorPosition / 2048);

        double rightRawSensorPosition = mCLMR0.getSelectedSensorPosition();
        double rightRotations = (rightRawSensorPosition / 2048);

        db.hanger.set(EHangerModuleData.L_VEL_rpm,leftRotationsPerMinute);
        db.hanger.set(EHangerModuleData.R_VEL_rpm, rightRotationsPerMinute);
        db.hanger.set(EHangerModuleData.L_POSITION_rot, leftRotations);
        db.hanger.set(EHangerModuleData.R_POSITION_rot, rightRotations);
    }

    @Override
    public void setOutputs() {
        Enums.EHangerMode motorMode = db.hanger.get(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.class);
        if (motorMode == null) {
            motorMode = Enums.EHangerMode.DEFAULT;
        }
        switch(motorMode) {
            case VELOCITY:
                mCLL0.set(ControlMode.Velocity, db.hanger.get(EHangerModuleData.L_DESIRED_VEL_rpm));
                mCLMR0.set(ControlMode.Velocity, db.hanger.get(EHangerModuleData.R_DESIRED_VEL_rpm));
                break;
            case DEFAULT:
                mCLL0.set(ControlMode.Velocity, 0);
                mCLMR0.set(ControlMode.Velocity, 0);
                break;
            case PERCENT_OUTPUT :
                mCLL0.set(ControlMode.PercentOutput, db.hanger.get(EHangerModuleData.L_SET_pct));
                mCLMR0.set(ControlMode.PercentOutput, db.hanger.get(EHangerModuleData.R_SET_pct));
                break;
            case POSITION:
                mCLL0.set(ControlMode.Position, db.hanger.get(EHangerModuleData.L_DESIRED_POSITION_rot));
                mCLMR0.set(ControlMode.Position, db.hanger.get(EHangerModuleData.R_DESIRED_POSITION_rot));
                break;
        }

        Enums.EHangerPneumaticMode pneumaticMode = db.hanger.get(EHangerModuleData.PNEUMATIC_STATE, Enums.EHangerPneumaticMode.class);
        if (pneumaticMode == null) {
            pneumaticMode = Enums.EHangerPneumaticMode.DEFAULT;
        }
        switch (pneumaticMode) {
            case LEFT_CLAMPED:
                mCLPNL.set(DoubleSolenoid.Value.kForward);
                break;
            case RIGHT_CLAMPED:
                mCLPNR.set(DoubleSolenoid.Value.kForward);
                break;
            case LEFT_RELEASED:
                mCLPNL.set(DoubleSolenoid.Value.kReverse);
                break;
            case RIGHT_RELEASED:
                mCLPNR.set(DoubleSolenoid.Value.kReverse);
                break;
            case DEFAULT:
                mCLPNL.set(DoubleSolenoid.Value.kReverse);
                mCLPNR.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }
}
