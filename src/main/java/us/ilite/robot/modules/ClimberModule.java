package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16/42);
    public static final double kMaxFalconSpeed = 6380 * kClimberRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kClimberRatio;

    public ClimberModule() {
        mCLMR11 = new TalonFX(11);
        mCL12 = new TalonFX(12);
        mCLMR11.setNeutralMode(NeutralMode.Brake);
        mCL12.setNeutralMode(NeutralMode.Brake);
        mCL12.configOpenloopRamp(0.5);
        mCLMR11.configOpenloopRamp(0.5);
        //Add 10 ms for current limit
        mCL12.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));
        mCLMR11.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 19, 20, 0.01));

        mCLPNA = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNB = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void readInputs() {
        db.climber.set(EClimberModuleData.L_VEL_rpm, mCL12.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.R_VEL_rpm, mCLMR11.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.L_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(EClimberModuleData.R_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
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
