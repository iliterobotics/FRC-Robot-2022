package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;

import us.ilite.robot.hardware.DigitalBeamSensor;

import static us.ilite.common.types.EFeederData.*;


public class FeederModule extends Module {

    private final TalonFX mIntakeFeeder;

    private final DigitalBeamSensor mEntryBeamBreaker;

    // ========================================
    // DO NOT MODIFY THESE METHOD CONSTANTS
    // ========================================
    public static final double kFeederGearRatio = (12.0 / 64.0) * (30.0 / 80.0);
    public static final double kFeederWheelDiameterInches = 2.5 / 12.0;
    public static final double kWheelCircumference = kFeederWheelDiameterInches * Math.PI;
    public static final double kDebounceTime = 0.1;
    public static final double kScaledRPMConversion = 600.0 / 2048.0 * kFeederGearRatio;
    public static final double kVelocityConversion = (kScaledRPMConversion * kWheelCircumference) / 60.0;
    public static final double kMaxFalconSpeed = 6380 * kFeederGearRatio;

    public FeederModule () {
        mIntakeFeeder = new TalonFX(Settings.HW.CAN.kINFeeder);
        mIntakeFeeder.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
        mEntryBeamBreaker = new DigitalBeamSensor(Settings.HW.DIO.kINEntryBeam, kDebounceTime);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        db.feeder.set(NUM_BALLS, 0);
    }

    @Override
    protected void readInputs() {
        db.feeder.set(ACTUAL_FEEDER_pct, (mIntakeFeeder.getSelectedSensorVelocity() * kScaledRPMConversion) / kMaxFalconSpeed);
        db.feeder.set(EXIT_BALL_VELOCITY_ft_s, mIntakeFeeder.getSelectedSensorVelocity() * kVelocityConversion);
        db.feeder.set(ENTRY_BEAM, mEntryBeamBreaker.isBroken());
    }

    @Override
    protected void setOutputs() {
        mIntakeFeeder.set(TalonFXControlMode.PercentOutput, db.feeder.get(SET_FEEDER_pct));
    }
}