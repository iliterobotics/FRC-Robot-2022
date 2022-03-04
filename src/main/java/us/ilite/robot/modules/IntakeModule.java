package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;

public class IntakeModule extends Module {
    private final TalonFX mIntakeRoller;
    private final DoubleSolenoid mArmSolenoid;
    private final Compressor mCompressor;

    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double kIntakeRollerRatio = (1.0 / 4.0) * (24.0 / 32.0);
    public static final double kMaxFalconSpeed = 6380 * kIntakeRollerRatio;
    public static final double kWheelDiameter = 2.0 / 12.0;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kIntakeRollerRatio;
    public static final double kFeetSpeedConversion = (kScaledUnitsToRPM * kWheelCircumference) / 60.0;

    public IntakeModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kINRoller);
        mIntakeRoller.setInverted(true);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
        mArmSolenoid = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, Settings.HW.PCH.kINPNIntakeForward, Settings.HW.PCH.kINPNIntakeReverse);
        mCompressor = new Compressor(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH);
        mCompressor.enableAnalog(100, 110);
    }

    @Override
    public void readInputs() {
        db.intake.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kFeetSpeedConversion);
        db.intake.set(FEEDER_pct, (mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM) / kMaxFalconSpeed);
        db.intake.set(CURRENT_ROLLER_RPM, mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.intake.set(INTAKE_SUPPLY_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.intake.set(INTAKE_STATOR_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.intake.set(COMPRESSOR_PSI, mCompressor.getPressure());
    }

    @Override
    public void setOutputs() {
       setPneumaticState();
       setRollerState();
    }

    public void setPneumaticState() {
        Enums.EArmState mode = db.intake.get(ARM_STATE, Enums.EArmState.class);
        if (mode == null) {
           return;
        }
        switch (mode) {
            case DEFAULT:
                mArmSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            case RETRACT:
                mArmSolenoid.set(DoubleSolenoid.Value.kForward);
                break;
        }
    }

    public void setRollerState() {
        Enums.ERollerState mode = db.intake.get(ROLLER_STATE, Enums.ERollerState.class);
        if (mode == null) {
            mode = Enums.ERollerState.PERCENT_OUTPUT;
        }
        switch (mode) {
            case PERCENT_OUTPUT:
                mIntakeRoller.set(TalonFXControlMode.PercentOutput, db.intake.get(DESIRED_pct));
                break;
        }
    }
}