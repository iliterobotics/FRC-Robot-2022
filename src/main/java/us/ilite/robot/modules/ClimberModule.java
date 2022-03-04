package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EClimberModuleData;

public class ClimberModule extends Module{
    private TalonFX mCL12;
    private TalonFX mCLMR11;
    private DoubleSolenoid mCLPNA;
    private DoubleSolenoid mCLPNB;

    public ClimberModule() {
        mCLMR11 = new TalonFX(11);
        mCL12 = new TalonFX(12);
        mCLMR11.setNeutralMode(NeutralMode.Brake);
        mCL12.setNeutralMode(NeutralMode.Brake);
        mCL12.configOpenloopRamp(2);
        mCLMR11.configOpenloopRamp(2);

        mCLPNA = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNB = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void readInputs() {
        //convert from raw sensor velocity inputs to rpm
        double leftRawSensorVelocity = mCL12.getSelectedSensorVelocity();
        double leftRotationsPerSecond = (leftRawSensorVelocity / 2048) / 1000;
        double leftRotationsPerMinute = leftRotationsPerSecond * 60;

        double rightRawSensorVelocity = mCLMR11.getSelectedSensorVelocity();
        double rightRotationsPerSecond = (rightRawSensorVelocity / 2048) / 1000;
        double rightRotationsPerMinute = rightRotationsPerSecond * 60;

        //convert from raw sensor position inputs rpm
        double leftRawSensorPosition = mCL12.getSelectedSensorPosition();
        double leftRotations = (leftRawSensorPosition / 2048);

        double rightRawSensorPosition = mCLMR11.getSelectedSensorPosition();
        double rightRotations = (rightRawSensorPosition / 2048);

        db.climber.set(EClimberModuleData.L_VEL_rpm,leftRotationsPerMinute);
        db.climber.set(EClimberModuleData.R_VEL_rpm, rightRotationsPerMinute);
        db.climber.set(EClimberModuleData.L_POSITION_rot, leftRotations);
        db.climber.set(EClimberModuleData.R_POSITION_rot, rightRotations);
    }


    @Override
    public void setOutputs() {
        mCL12.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.L_SET_pct));
        mCLMR11.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.R_SET_pct));

        double isAClamped = db.climber.get(EClimberModuleData.IS_A_CLAMPED);
        double isBClamped = db.climber.get(EClimberModuleData.IS_B_CLAMPED);
        if(isAClamped == 1.0) {
            mCLPNA.set(DoubleSolenoid.Value.kForward);
        }
        else if (isAClamped == 2.0) {
            mCLPNA.set(DoubleSolenoid.Value.kReverse);
        }

        if (isBClamped == 1.0) {
            mCLPNB.set(DoubleSolenoid.Value.kForward);
        } else if (isBClamped == 2.0) {
            mCLPNB.set(DoubleSolenoid.Value.kReverse);
        }

    }
}
