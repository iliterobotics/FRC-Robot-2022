package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.lib.util.Utils;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import static us.ilite.common.types.EShooterSystemData.*;

public class FlywheelModule extends Module {

    public enum EHoodState {
        NONE,
        TARGET_ANGLE,
        MANUAL
    }

    public enum EHoodSensorError {
        NONE,
        INVALID_POTENTIOMETER_READING
    }

    public enum EFlywheelWheelState {
        NONE,
        OPEN_LOOP,
        VELOCITY,
    }

    // Flywheel Motors
    private TalonFX mFlywheelFalconMaster;
    private TalonFX mFlywheelFalconFollower;
    private CANSparkMax mFeeder;
    private TalonSRX mTurret;
    private ContinuousRotationServo mHoodServo;

    private AnalogInput mHoodAI;
    private AnalogPotentiometer mHoodPot;

    // Constants
    private static final double kHoodGearRatio = 32.0 / 20.0 * 14.0 / 360.0;//First stage was 32/20 in code but 36/20 in integration
    private static final double kTurretGearRatio = 9.0 / 72.0 * 18.0 / 160.0;
    private static final double kFlyWheelGearRatio = 36.0 / 24.0;
    private static final double kFlywheelDiameterInches = 4.0;
    private static double kFlywheelRadiusFeet = kFlywheelDiameterInches /2.0 / 12.0;


    // Production bot 0.00175 min, 0.74664 max (normalized from AnalogPotentiometer)
    private static final double kHoodReadingRange = 0.74664 - 0.00175;
    private static final double kMinShotDegrees = 20.0;
    private static final double kMaxShotDegrees = 80.0;
    private static final double kHoodAngleRange = kMaxShotDegrees - kMinShotDegrees;
    private static final double kHoodConversionFactor = kHoodReadingRange / kHoodAngleRange;
    private final FilteredAverage mPotentiometerReadings = FilteredAverage.filteredAverageForTime(0.1);


    private static double kRadiansPerSecToTalonTicksPer100ms = (2*Math.PI) / 2048.0 / 0.1;
    // https://docs.google.com/spreadsheets/d/1Po6exzGvfr0rMWMWVSyqxf49ZMSfGoYn/edit#gid=1858991275
    private static double kVelocityConversion = (2048.0 / 600.0) / (kFlyWheelGearRatio * kFlywheelDiameterInches * Math.PI / 12.0 / 60.0);

    private static final int FLYWHEEL_SLOT = 0;
    private static ProfileGains kFlywheelGains = new ProfileGains()
            .slot(FLYWHEEL_SLOT)
            .p(0.01)
            .f(0.25)
            // Units are RADIANS
//            .p(0.0497)
//            .f(0.018)
//            .kA(0.00165)
//            .kV(0.018)
            ;
    private PIDController mHoodPID = new PIDController(5, 0, 0);


    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        SmartDashboard.putNumber("kRadiansPerSecToTalonTicksPer100ms", kRadiansPerSecToTalonTicksPer100ms);
        SmartDashboard.putNumber("kVelocityConversion",kVelocityConversion);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        /* config all the settings */
        mFlywheelFalconMaster = new TalonFX(Settings.Hardware.CAN.kFalconMasterId);
        mFlywheelFalconMaster.configFactoryDefault();
        mFlywheelFalconMaster.configAllSettings(configs);
        mFlywheelFalconMaster.setNeutralMode(NeutralMode.Coast);

        mFlywheelFalconFollower = new TalonFX(Settings.Hardware.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.configFactoryDefault();
        mFlywheelFalconFollower.configAllSettings(configs);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Coast);
        mFlywheelFalconFollower.setInverted(true);
        HardwareUtils.setGains(mFlywheelFalconFollower, kFlywheelGains);

        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);

        mTurret = TalonSRXFactory.createDefaultTalon(Settings.Hardware.CAN.kSRXTurretId);

//        mHoodServo = new ContinuousRotationServo(Settings.Hardware.PWM.kHoodServoId).inverted(true);

        mHoodAI = new AnalogInput(Settings.Hardware.Analog.kHoodPot);
        mHoodPot = new AnalogPotentiometer(mHoodAI);

    }
    
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        //Set PID gains & zero encoders here
        HardwareUtils.setGains(mFlywheelFalconMaster, kFlywheelGains);
        HardwareUtils.setGains(mFlywheelFalconFollower, kFlywheelGains);
    }

    @Override
    public void readInputs(double pNow) {
        db.flywheel.set(CURRENT_FLYWHEEL_VELOCITY, mFlywheelFalconMaster.getSelectedSensorVelocity() / kVelocityConversion);
        db.flywheel.set(CURRENT_FEEDER_VELOCITY, mFeeder.get());
        db.flywheel.set(CURRENT_HOOD_ANGLE, mHoodPot.get() / kHoodReadingRange * kHoodAngleRange + kMinShotDegrees);
        db.flywheel.set(POT_NORM_VALUE, mHoodPot.get());
        db.flywheel.set(POT_RAW_VALUE, mHoodAI.getValue());
//        db.flywheel.set(HOOD_SERVO_RAW_VALUE, mHoodServo.getRawOutputValue());
//        db.flywheel.set(HOOD_SERVO_LAST_VALUE, mHoodServo.getLastValue());
        mPotentiometerReadings.addNumber(mHoodAI.getValue());

        double cur = mHoodAI.getValue();
        double avg = mPotentiometerReadings.getAverage();
        if(
                mHoodServo != null &&
                Utils.isWithinTolerance(avg, cur,0.5) &&
                Math.abs(mHoodServo.getLastValue()) > 0.1
        ) {
            db.flywheel.set(HOOD_SENSOR_ERROR, EHoodSensorError.INVALID_POTENTIOMETER_READING);
        } else {
            db.flywheel.set(HOOD_SENSOR_ERROR, EHoodSensorError.NONE);
        }
        db.flywheel.set(POT_AVG_VALUE, avg);

//        if (db.limelight.isSet(ELimelightData.TV)) {
//            db.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
//            db.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
//        } else {
//            db.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
//            db.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, 0);
//        }
    }

    @Override
    public void setOutputs(double pNow) {
//        mFlywheelFalconMaster.set(ControlMode.Velocity, db.flywheel.get(TARGET_FLYWHEEL_VELOCITY));
//        mFlywheelFalconFollower.set(ControlMode.Velocity, db.flywheel.get(TARGET_FLYWHEEL_VELOCITY));

//        mFeeder.set(db.flywheel.get(TARGET_FEEDER_VELOCITY));

        setFlywheel();
//        setTurret();
//        setHood(pNow);

//        hoodAnglePID(0.005, 0, 0);
    }

    @Override
    public void shutdown(double pNow) {
        db.flywheel.set(TARGET_FLYWHEEL_VELOCITY, 0);
        db.flywheel.set(TARGET_FEEDER_VELOCITY, 0);
        db.flywheel.set(TARGET_HOOD_ANGLE, 0);
    }

    private boolean isPastVelocity(double pVelocity) {
        // TODO Convert ticks per 100 ms to rpm
        return mFlywheelFalconMaster.getSelectedSensorVelocity() >= pVelocity;
    }

    private void setFlywheel() {
        EFlywheelWheelState state = db.flywheel.get(FLYWHEEL_WHEEL_STATE, EFlywheelWheelState.class);
        if(state == null) state = EFlywheelWheelState.NONE;
        switch (state) {
            case OPEN_LOOP:
                double l = db.flywheel.get(FLYWHEEL_OPEN_LOOP);
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, l);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, l);
                break;
            case VELOCITY:
                if(db.flywheel.isSet(TARGET_FLYWHEEL_VELOCITY)){
                    double flywheelOutput = db.flywheel.get(TARGET_FLYWHEEL_VELOCITY) * kVelocityConversion;
                    SmartDashboard.putNumber("Flywheel Raw Output", flywheelOutput);
                    mFlywheelFalconMaster.selectProfileSlot(FLYWHEEL_SLOT, 0);
                    mFlywheelFalconMaster.set(TalonFXControlMode.Velocity, flywheelOutput);
//                    mFlywheelFalconFollower.selectProfileSlot(FLYWHEEL_SLOT, 0);
//                    mFlywheelFalconFollower.set(TalonFXControlMode.Velocity, flywheelOutput);
                } else {
                    mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, 0.0);
                    mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, 0.0);
                }
            case NONE:
            default:
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, 0.0);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    private double setHood(double pNow) {
        EHoodState state = db.flywheel.get(HOOD_STATE, EHoodState.class);
        if(state == null) state = EHoodState.NONE;

        switch(state) {
            case MANUAL:
                mHoodServo.setServo(db.flywheel.get(FLYWHEEL_TEST));
                break;
            case TARGET_ANGLE:
                double target = db.flywheel.get(TARGET_HOOD_ANGLE)* kHoodConversionFactor;
                double current = db.flywheel.get(CURRENT_HOOD_ANGLE)* kHoodConversionFactor;

                double output = mHoodPID.calculate(current, target);
                mHoodServo.setServo(output);
                break;
            case NONE:
            default:
                mHoodServo.setServo(0.0);

        }
        return 0.0;
    }

//    private double calcSpeedFromDistance(Distance pDistance) {
//        //TODO figure out necessity
//        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
//                - 0.209 * Math.pow(pDistance.inches(), 2)
//                + 6.31 * pDistance.inches()
//                + 227;
//    }
//
//    private double calcAngleFromDistance(Distance pDistance) {
//        //TODO tuning
//        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
//                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
//                + 0.157 * Math.pow(pDistance.inches(), 2)
//                + 2.94 * pDistance.inches()
//                + 68.2;
//    }

    private double hoodAnglePID(double pP, double pI, double pD) {
        // TODO Convert into an Angle Measurement
        double error = pP * (db.flywheel.get(TARGET_HOOD_ANGLE) - db.flywheel.get(CURRENT_HOOD_ANGLE));
        double velocity = pD * mHoodServo.getSpeed();
        mIntegral += pI * mHoodServo.getSpeed();
        return error + mIntegral + velocity;
    }

    private void potentiometerPID() {
        double output = db.flywheel.get(TARGET_HOOD_ANGLE) / 5;
    }
}