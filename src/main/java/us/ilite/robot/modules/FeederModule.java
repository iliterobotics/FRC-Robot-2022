package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;

import us.ilite.robot.hardware.DigitalBeamSensor;

import static us.ilite.common.types.EFeederData.*;


public class FeederModule extends Module {

    private final TalonFX mIntakeFeeder;

    private final DigitalBeamSensor mEntryBeamBreaker;
    private final DigitalBeamSensor mExitBeamBreaker;

    //Constants
    private final double kWheelCircumference = 4 * Math.PI;
    private final double kDebounceTime = 0.1;
    private final double kVelocityConversion = 2048 * 1000 * kWheelCircumference;
    private final double kMaxFalconSpeed = 6380;

    public FeederModule () {
        mIntakeFeeder = new TalonFX(Settings.HW.CAN.kINFeeder);
        mEntryBeamBreaker = new DigitalBeamSensor(Settings.HW.DIO.kINEntryBeam, kDebounceTime);
        mExitBeamBreaker = new DigitalBeamSensor(Settings.HW.DIO.kINExitBeam, kDebounceTime);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        db.feeder.set(NUM_BALLS, 0);
    }

    @Override
    public void readInputs() {
        db.feeder.set(FEEDER_pct, mIntakeFeeder.getSelectedSensorVelocity()* kVelocityConversion / kMaxFalconSpeed);
        db.feeder.set(ENTRY_BEAM, mEntryBeamBreaker.isBroken());
        db.feeder.set(EXIT_BEAM, mExitBeamBreaker.isBroken());
    }

    @Override
    public void setOutputs() {
        mIntakeFeeder.set(TalonFXControlMode.PercentOutput, db.feeder.get(SET_FEEDER_pct));
    }
}