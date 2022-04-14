package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EMatchMode;

import us.ilite.robot.Enums;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;

import static us.ilite.common.types.EFeederData.*;


public class FeederModule extends Module {

    private final TalonFX mIntakeFeeder;

    private final DigitalBeamSensor mEntryBeamBreaker;
    private final DigitalBeamSensor mExitBeamBreaker;

    // ========================================
    // DO NOT MODIFY THESE HARDWARE CONSTANTS
    // ========================================
    public static final double kFeederGearRatio = (12.0 / 64.0) * (30.0 / 80.0);
    public static final double kFeederWheelDiameterInches = 2.5 / 12.0;
    public static final double kWheelCircumference = kFeederWheelDiameterInches * Math.PI;
    public static final double kDebounceTime = 0.1;
    public static final double kScaledRPMConversion = 600.0 / 2048.0 * kFeederGearRatio;
    public static final double kVelocityConversion = (kScaledRPMConversion * kWheelCircumference) / 60.0;
    public static final double kMaxFalconSpeed = 6380 * kFeederGearRatio;

    private int VELOCITY_SLOT = 0;
    private ProfileGains kVelocityGains = new ProfileGains().p(0.0001).f(0.0001).slot(VELOCITY_SLOT);

    public FeederModule () {
        mIntakeFeeder = new TalonFX(Settings.HW.CAN.kINFeeder);
        mEntryBeamBreaker = new DigitalBeamSensor(Settings.HW.DIO.kINEntryBeam, kDebounceTime);
        mExitBeamBreaker = new DigitalBeamSensor(Settings.HW.DIO.kINExitBeam, kDebounceTime);
        mIntakeFeeder.configPeakOutputForward(1.0, 20);
        mIntakeFeeder.configPeakOutputReverse(1.0, 20);
        setStatusFrames();
        HardwareUtils.setGains(mIntakeFeeder, kVelocityGains);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        db.feeder.set(NUM_BALLS, 0);
    }

    @Override
    public void readInputs() {
        db.feeder.set(ACTUAL_FEEDER_pct, (mIntakeFeeder.getSelectedSensorVelocity() * kScaledRPMConversion) / kMaxFalconSpeed);
        db.feeder.set(EXIT_BALL_VELOCITY_ft_s, mIntakeFeeder.getSelectedSensorVelocity() * kVelocityConversion);
        db.feeder.set(ENTRY_BEAM, mEntryBeamBreaker.isBroken());
        db.feeder.set(EXIT_BEAM, mExitBeamBreaker.isBroken());
    }

    @Override
    public void setOutputs() {
        Enums.EFeederState mode = db.feeder.get(STATE, Enums.EFeederState.class);
        if (mode == null) {
            return;
        }
        switch (mode) {
            case PERCENT_OUTPUT:
                mIntakeFeeder.set(TalonFXControlMode.PercentOutput, db.feeder.get(SET_FEEDER_pct));
                break;
            case VELOCITY:
                mIntakeFeeder.set(TalonFXControlMode.Velocity, rpmToTicksPer100ms(db.feeder.get(SET_VELOCITY_rpm)));
                break;
        }
    }
    private void setStatusFrames() {
        mIntakeFeeder.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        mIntakeFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    }
    private double rpmToTicksPer100ms(double pRPM) {
        return pRPM / kScaledRPMConversion;
    }
}