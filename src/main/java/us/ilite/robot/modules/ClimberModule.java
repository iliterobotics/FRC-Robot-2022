package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EClimberModuleData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

public class ClimberModule extends Module{
    private TalonFX mCL12;
    private TalonFX mCLMR11;
    private DoubleSolenoid mCLPNDouble;
    private DoubleSolenoid mCLPNSingle;

    private PIDController mPositionPID;
    private PIDController mVelocityPID;

    // ========================================
    // DO NOT MODIFY THESE PID CONSTANTS
    // ========================================

    private ProfileGains kVelocityGains = new ProfileGains().p(0.0).f(0.0001);
    private ProfileGains kPositionGains = new ProfileGains().p(0.0001);

    // ========================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS
    // ========================================

    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public static final double kMaxClimberSpeed = 6380 * kClimberRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kClimberRatio;
    public static final double kVerticalAngleDeg = 90;
    public static final double kMaxAllowedVelocity = 0.75;
    public static final double kMaxScaledVelocity = kMaxClimberSpeed * kMaxAllowedVelocity;

    public ClimberModule() {
        mCLMR11 = new TalonFX(Settings.HW.CAN.kCLM1);
        mCL12 = new TalonFX(Settings.HW.CAN.kCL2);
        mCLMR11.setNeutralMode(NeutralMode.Brake);
        mCL12.setNeutralMode(NeutralMode.Brake);
        mCL12.configOpenloopRamp(0.5);
        mCLMR11.configOpenloopRamp(0.5);

        //Add 10 ms for current limit
        mCL12.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));
        mCLMR11.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));

        mCLPNDouble = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        mVelocityPID = new PIDController(kVelocityGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);
        mPositionPID = new PIDController(kPositionGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        mPositionPID.setSetpoint(kVerticalAngleDeg);
        mPositionPID.setOutputRange(-kMaxScaledVelocity, kMaxScaledVelocity);
        mPositionPID.setContinuous(true);

        mVelocityPID.setSetpoint(0d);
        mVelocityPID.setOutputRange(-kMaxScaledVelocity, kMaxScaledVelocity);
    }

    @Override
    public void readInputs() {
        db.climber.set(EClimberModuleData.L_VEL_rpm, mCL12.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.R_VEL_rpm, mCLMR11.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.L_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(EClimberModuleData.R_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(EClimberModuleData.BUS_VOLTAGE_LEFT, mCL12.getBusVoltage());
        db.climber.set(EClimberModuleData.BUS_VOLTAGE_RIGHT, mCLMR11.getBusVoltage());
    }

    @Override
    public void setOutputs() {
        Enums.EClimberMode mode = db.climber.get(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberModuleData.IS_COAST)) {
            mCL12.setNeutralMode(NeutralMode.Coast);
            mCLMR11.setNeutralMode(NeutralMode.Coast);
        } else {
            mCL12.setNeutralMode(NeutralMode.Brake);
            mCLMR11.setNeutralMode(NeutralMode.Brake);
        }

        switch(mode) {
            case PERCENT_OUTPUT:
                mCL12.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.DESIRED_VEL_pct));
                mCLMR11.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.DESIRED_VEL_pct));
                break;
            case VELOCITY:
                double desiredVel = mVelocityPID.calculate(db.climber.get(EClimberModuleData.L_VEL_rpm), clock.getCurrentTimeInMillis());
                mCL12.set(ControlMode.Velocity, desiredVel);
                mCLMR11.set(ControlMode.Velocity, desiredVel);
                break;
            case POSITION:
                double desiredPos = mPositionPID.calculate(db.climber.get(EClimberModuleData.L_POSITION_deg), clock.getCurrentTimeInMillis());
                mCL12.set(ControlMode.Velocity, desiredPos);
                mCLMR11.set(ControlMode.Velocity, desiredPos);
                break;
        }

        if (db.climber.isSet(EClimberModuleData.DOUBLE_CLAMPED)) {
            mCLPNDouble.set(DoubleSolenoid.Value.kForward);
        } else {
            mCLPNDouble.set(DoubleSolenoid.Value.kReverse);
        }

        if (db.climber.isSet(EClimberModuleData.SINGLE_CLAMPED)) {
            mCLPNSingle.set(DoubleSolenoid.Value.kForward);
        } else {
            mCLPNSingle.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
