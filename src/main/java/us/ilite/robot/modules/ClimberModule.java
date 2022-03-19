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
import us.ilite.robot.hardware.HardwareUtils;

import static us.ilite.common.types.EClimberModuleData.*;

public class ClimberModule extends Module{
    private final TalonFX mCL12;
    private final TalonFX mCLMR11;
    private final DoubleSolenoid mCLPNDouble;
    private final DoubleSolenoid mCLPNSingle;

    private PIDController mVelocityPID;

    // ========================================
    // DO NOT MODIFY THESE PID CONSTANTS
    // ========================================

    private final int POSITION_SLOT = 0;

    private ProfileGains kVelocityGains = new ProfileGains().p(0.0).f(0.0001);
    private ProfileGains kPositionGains = new ProfileGains().p(0.015).f(0.0005).slot(POSITION_SLOT);

    // ========================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS
    // ========================================

    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public static final double kMaxClimberSpeed = 6380 * kClimberRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kClimberRatio;

    public ClimberModule() {
        mCLMR11 = new TalonFX(Settings.HW.CAN.kCLM1);
        mCL12 = new TalonFX(Settings.HW.CAN.kCL2);
        mCLMR11.setNeutralMode(NeutralMode.Brake);
        mCL12.setNeutralMode(NeutralMode.Brake);
        mCL12.configClosedloopRamp(0.5);
        mCLMR11.configClosedloopRamp(0.5);

        //Add 10 ms for current limit
        mCL12.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));
        mCLMR11.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));

        mCLPNDouble = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 20);

        mCLMR11.configAllowableClosedloopError(1, climberDegreesToTicks(2));
        mCL12.configAllowableClosedloopError(1, climberDegreesToTicks(2));

        mVelocityPID = new PIDController(kVelocityGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);

        mCL12.configNominalOutputForward(0, 20);
        mCLMR11.configNominalOutputForward(0, 20);

        mCL12.configNominalOutputReverse(0, 20);
        mCLMR11.configNominalOutputReverse(0, 20);

        mCL12.configPeakOutputForward(1, 20);
        mCLMR11.configPeakOutputForward(1, 20);

        mCL12.configPeakOutputReverse(-1, 20);
        mCLMR11.configPeakOutputReverse(-1, 20);

        mCL12.configMotionAcceleration(10 / kScaledUnitsToRPM, 20);
        mCLMR11.configMotionAcceleration(10 / kScaledUnitsToRPM, 20);

        HardwareUtils.setGains(mCL12, kPositionGains);
        HardwareUtils.setGains(mCLMR11, kPositionGains);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        if (mode == EMatchMode.TELEOPERATED) {
            mCLMR11.configClearPositionOnQuadIdx(true, 20);
            db.climber.set(L_POSITION_deg, ticksToClimberDegrees(mCL12.getSelectedSensorPosition()));
        }
    }

    @Override
    public void readInputs() {
        db.climber.set(L_VEL_rpm, mCL12.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(L_POSITION_deg, ticksToClimberDegrees(mCL12.getSelectedSensorPosition()));
        db.climber.set(L_POSITION_TARGET, mCL12.getClosedLoopTarget());
        db.climber.set(L_POSITION_ERROR, mCL12.getClosedLoopError());
        db.climber.set(L_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(BUS_VOLTAGE_LEFT, mCL12.getBusVoltage());
    }

    @Override
    public void setOutputs() {
        Enums.EClimberMode mode = db.climber.get(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberModuleData.SET_COAST)) {
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
                mCL12.set(ControlMode.Position, climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                mCLMR11.set(ControlMode.Position, climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                break;
        }

        Enums.EClampMode doubleMode = db.climber.get(IS_DOUBLE_CLAMPED, Enums.EClampMode.class);
        Enums.EClampMode singleMode = db.climber.get(IS_SINGLE_CLAMPED, Enums.EClampMode.class);

        if (doubleMode == Enums.EClampMode.CLAMPED) {
            mCLPNDouble.set(DoubleSolenoid.Value.kForward);
        } else if (doubleMode == Enums.EClampMode.RELEASED) {
            mCLPNDouble.set(DoubleSolenoid.Value.kReverse);
        }

        if (singleMode == Enums.EClampMode.CLAMPED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kForward);
        } else if (singleMode == Enums.EClampMode.RELEASED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kReverse);
        }
    }

    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * kClimberRatio * 360;
    }
    private double climberDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / kClimberRatio / 360;
    }
}
