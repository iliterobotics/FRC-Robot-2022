package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EClimberModuleData;
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

        mCLPNL = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNR = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCLL0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR0.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void readInputs() {
        mCLPNL.get();
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

        db.climber.set(EClimberModuleData.L_VEL_rpm,leftRotationsPerMinute);
        db.climber.set(EClimberModuleData.R_VEL_rpm, rightRotationsPerMinute);
        db.climber.set(EClimberModuleData.L_POSITION_rot, leftRotations);
        db.climber.set(EClimberModuleData.R_POSITION_rot, rightRotations);
    }


    @Override
    public void setOutputs() {
        int tmp = (int)db.climber.safeGet(EClimberModuleData.HANGER_STATE, 0d);
        Enums.EClimberMode motorMode = Enums.EClimberMode.values()[tmp];
        if (motorMode == null) {
            motorMode = Enums.EClimberMode.PERCENT_OUTPUT;
        }
        mCLL0.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.L_SET_pct));
        mCLMR0.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.R_SET_pct));
//        switch(motorMode) {
//            case VELOCITY:
//                mCLL0.set(ControlMode.Velocity, db.climber.get(EClimberModuleData.L_DESIRED_VEL_rpm));
//                mCLMR0.set(ControlMode.Velocity, db.climber.get(EClimberModuleData.R_DESIRED_VEL_rpm));
//                break;
//            case DEFAULT:
//                mCLL0.set(ControlMode.Velocity, 0);
//                mCLMR0.set(ControlMode.Velocity, 0);
//                break;
//            case PERCENT_OUTPUT :
//                mCLL0.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.L_SET_pct));
//                mCLMR0.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.R_SET_pct));
//                break;
//            case POSITION:
//                mCLL0.set(ControlMode.Position, db.climber.get(EClimberModuleData.L_DESIRED_POSITION_rot));
//                mCLMR0.set(ControlMode.Position, db.climber.get(EClimberModuleData.R_DESIRED_POSITION_rot));
//                break;

//        }
        Enums.EHangerPneumaticMode pneumaticMode = db.climber.get(EClimberModuleData.PNEUMATIC_STATE, Enums.EHangerPneumaticMode.class);
        if (pneumaticMode == null) {
            pneumaticMode = Enums.EHangerPneumaticMode.DEFAULT;
        }
//        switch (pneumaticMode) {
//            case TOP_CLAMPED:
//                mCLPNL.set(DoubleSolenoid.Value.kForward);
//                break;
//            case BOTTOM_CLAMPED:
//                mCLPNR.set(DoubleSolenoid.Value.kForward);
//                break;
//            case TOP_RELEASED:
//                mCLPNL.set(DoubleSolenoid.Value.kReverse);
//                break;
//            case BOTTOM_RELEASED:
//                mCLPNR.set(DoubleSolenoid.Value.kReverse);
//                break;
//            case DEFAULT:
//                mCLPNL.set(DoubleSolenoid.Value.kReverse);
//                mCLPNR.set(DoubleSolenoid.Value.kReverse);
//                break;
//        }
        System.out.println("MODE: " + pneumaticMode);
        if(pneumaticMode == Enums.EHangerPneumaticMode.TOP_CLAMPED) {
            mCLPNL.set(DoubleSolenoid.Value.kForward);
        } else {
            mCLPNL.set(DoubleSolenoid.Value.kReverse);
        }

        if(pneumaticMode == Enums.EHangerPneumaticMode.BOTTOM_CLAMPED) {
            mCLPNR.set(DoubleSolenoid.Value.kForward);
        } else {
            mCLPNR.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
