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
    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public static final double kMaxFalconSpeed = 6380 * kClimberRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kClimberRatio;
    public static final double kScaledUnitsToRotations = (1.0 / 2048.0) * kClimberRatio;

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

        mCLPNA = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 2, 3);
        mCLPNB = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);

        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    protected void readInputs() {
        db.climber.set(EClimberModuleData.L_VEL_rpm, mCL12.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.R_VEL_rpm, mCLMR11.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(EClimberModuleData.L_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(EClimberModuleData.R_OUTPUT_CURRENT, mCLMR11.getSupplyCurrent());
        db.climber.set(EClimberModuleData.BUS_VOLTAGE_LEFT, mCL12.getBusVoltage());
        db.climber.set(EClimberModuleData.BUS_VOLTAGE_RIGHT, mCLMR11.getBusVoltage());
        db.climber.set(EClimberModuleData.L_DEGREES, mCL12.getSelectedSensorPosition() * kScaledUnitsToRotations * 360);
        db.climber.set(EClimberModuleData.R_DEGREES, mCLMR11.getSelectedSensorPosition() * kScaledUnitsToRotations * 360);
        db.climber.set(EClimberModuleData.L_ROTATIONS, mCL12.getSelectedSensorPosition() * kScaledUnitsToRotations);
        db.climber.set(EClimberModuleData.R_ROTATIONS, mCLMR11.getSelectedSensorPosition() * kScaledUnitsToRotations);
        //This is in celsius
        db.climber.set(EClimberModuleData.L_TEMPERATURE, mCL12.getTemperature());
        db.climber.set(EClimberModuleData.R_TEMPERATURE, mCLMR11.getTemperature());
    }


    @Override
    protected void setOutputs() {
        mCL12.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.L_SET_pct));
        mCLMR11.set(ControlMode.PercentOutput, db.climber.get(EClimberModuleData.R_SET_pct));

        if (db.climber.isSet(EClimberModuleData.IS_COAST)) {
            mCL12.setNeutralMode(NeutralMode.Coast);
            mCLMR11.setNeutralMode(NeutralMode.Coast);
        } else {
            mCL12.setNeutralMode(NeutralMode.Brake);
            mCLMR11.setNeutralMode(NeutralMode.Brake);
        }

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
