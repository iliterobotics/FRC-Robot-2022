package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.types.EMatchMode;
import static us.ilite.robot.Enums.*;
import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import static us.ilite.common.types.EShooterSystemData.*;

public class FlywheelModule extends Module {

    // Flywheel Motors
    private TalonFX mFlywheelFalconMaster;
    private TalonFX mFlywheelFalconFollower;
    private CANSparkMax mFeeder;
    private TalonSRX mTurret;
    private ContinuousRotationServo mHoodServo;

    // These were for direct RIO connections
//    private AnalogInput mHoodAI;
//    private AnalogPotentiometer mHoodPot;

    // This is for the feeder MAX connection
    private CANAnalog mHoodPot;

    // Constants
    private static final double kHoodGearRatio = 32.0 / 20.0 * 14.0 / 360.0;//First stage was 32/20 in code but 36/20 in integration
    private static final double kTurretGearRatio = 9.0 / 72.0 * 18.0 / 160.0;
    private static final double kFlyWheelGearRatio = 36.0 / 24.0;
    private static final double kFlywheelDiameterInches = 4.0;
    private static double kFlywheelRadiusFeet = kFlywheelDiameterInches /2.0 / 12.0;


    // Production bot 0.00175 min, 0.74664 max (normalized from AnalogPotentiometer)
//    private static final double kHoodReadingRange = 0.74664 - 0.00175;
    // Practice & Production have the same min reading
    private static final double kHoodMinReading = 0.00175;
    // Max reading taken from practice bot while connected to MAX - needs updating on production bot
    private static final double kHoodReadingRange = 1.637 - kHoodMinReading;
    private static final double kMinShotDegrees = 90.0 - 66.0;
    private static final double kMaxShotDegrees = 90.0 - 14.0;
    private static final double kHoodAngleRange = kMaxShotDegrees - kMinShotDegrees;
    private static final double kHoodConversionFactor = kHoodReadingRange / kHoodAngleRange;
    private final FilteredAverage mPotentiometerReadings = FilteredAverage.filteredAverageForTime(0.1);


    private static double kRadiansPerSecToTalonTicksPer100ms = (2*Math.PI) / 2048.0 / 0.1;
    // https://docs.google.com/spreadsheets/d/1Po6exzGvfr0rMWMWVSyqxf49ZMSfGoYn/edit#gid=1858991275
    // Converts from desired BALL (not wheel) velocity to Falcon ticks per 100ms
    private static double kMOISlipFactor = 1.25;
    private static double kVelocityConversion = kMOISlipFactor * (2048.0 / 600.0) / (kFlyWheelGearRatio * kFlywheelDiameterInches * Math.PI / 12.0 / 60.0) * 2.0;

    private static final int FLYWHEEL_SLOT = 0;
    private static ProfileGains kFlywheelGains = new ProfileGains()
            .slot(FLYWHEEL_SLOT)
            .p(0.05)
            .f(0.0575)
//            .kA(0.00165)
//            .kV(0.018)
            ;
    private PIDController mHoodPID = new PIDController(5, 0, 0);
    private CANEncoder mFeederInternalEncoder;

    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        SmartDashboard.putNumber("kRadiansPerSecToTalonTicksPer100ms", kRadiansPerSecToTalonTicksPer100ms);
        SmartDashboard.putNumber("kVelocityConversion",kVelocityConversion);
        mFlywheelFalconMaster = new TalonFX(Settings.Hardware.CAN.kFalconMasterId);
        mFlywheelFalconMaster.setInverted(true);
        mFlywheelFalconMaster.setNeutralMode(NeutralMode.Coast);
        mFlywheelFalconMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        mFlywheelFalconFollower = new TalonFX(Settings.Hardware.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Coast);
        mFlywheelFalconFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);
        mFeederInternalEncoder = new CANEncoder(mFeeder);
        mFeeder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mFeeder.setSmartCurrentLimit(30);


        mTurret = TalonSRXFactory.createDefaultTalon(Settings.Hardware.CAN.kSRXTurretId);

        mHoodServo = new ContinuousRotationServo(Settings.Hardware.PWM.kHoodServoId).inverted(true);
        mHoodPot = mFeeder.getAnalog(CANAnalog.AnalogMode.kAbsolute);

//        mHoodAI = new AnalogInput(Settings.Hardware.Analog.kHoodPot);
//        mHoodPot = new AnalogPotentiometer(mHoodAI);

    }
    
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        //Set PID gains & zero encoders here
        HardwareUtils.setGains(mFlywheelFalconMaster, kFlywheelGains);
        HardwareUtils.setGains(mFlywheelFalconFollower, kFlywheelGains);
    }

    @Override
    public void readInputs(double pNow) {
        db.flywheel.set(SET_BALL_VELOCITY_ft_s, mFlywheelFalconMaster.getSelectedSensorVelocity() / kVelocityConversion);
        db.flywheel.set(FEEDER_rpm, mFeederInternalEncoder.getVelocity());
        double position = mHoodPot.getPosition();
        db.flywheel.set(POT_RAW_VALUE, position);
        db.flywheel.set(CURRENT_HOOD_ANGLE, convertToHoodAngle(position));
//        db.flywheel.set(POT_NORM_VALUE, mHoodPot.get());
//        db.flywheel.set(POT_RAW_VALUE, mHoodAI.getValue());
        db.flywheel.set(HOOD_SERVO_RAW_VALUE, mHoodServo.getRawOutputValue());
        db.flywheel.set(HOOD_SERVO_LAST_VALUE, mHoodServo.getLastValue());
//        mPotentiometerReadings.addNumber(mHoodAI.getValue());

//        double cur = mHoodAI.getValue();
//        double avg = mPotentiometerReadings.getAverage();
//        if(
//                mHoodServo != null &&
//                Utils.isWithinTolerance(avg, cur,0.5) &&
//                Math.abs(mHoodServo.getLastValue()) > 0.1
//        ) {
////            db.flywheel.set(HOOD_SENSOR_ERROR, EHoodSensorError.INVALID_POTENTIOMETER_READING);
//            db.flywheel.set(HOOD_SENSOR_ERROR, EHoodSensorError.NONE);
//        } else {
//            db.flywheel.set(HOOD_SENSOR_ERROR, EHoodSensorError.NONE);
//        }
//        db.flywheel.set(POT_AVG_VALUE, avg);

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
        setFlywheel();
        setFeeder();
//        setTurret();
        setHood(pNow);
    }

    @Override
    public void shutdown(double pNow) {
//        db.flywheel.set(TARGET_BALL_VELOCITY, 0);
//        db.flywheel.set(TARGET_FEEDER_VELOCITY, 0);
//        db.flywheel.set(TARGET_HOOD_ANGLE, 0);
    }

    private void setFeeder() {
        // Cannot set voltage mode if an external sensor is attached
//        mFeeder.setVoltage(db.flywheel.get(FEEDER_OUTPUT_OPEN_LOOP) / 12.0);
        mFeeder.set(db.flywheel.get(SET_FEEDER_rpm));
    }

    private void setFlywheel() {
        FlywheelWheelState state = db.flywheel.get(FLYWHEEL_WHEEL_STATE, FlywheelWheelState.class);
        if(state == null) state = FlywheelWheelState.NONE;
        switch (state) {
            case OPEN_LOOP:
                double l = db.flywheel.get(FLYWHEEL_OPEN_LOOP);
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, l);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, l);
                break;
            case VELOCITY:
                double flywheelOutput = db.flywheel.get(BALL_VELOCITY_ft_s) * kVelocityConversion;
                SmartDashboard.putNumber("Flywheel Raw Output", flywheelOutput);
                mFlywheelFalconMaster.set(TalonFXControlMode.Velocity, flywheelOutput);
                mFlywheelFalconFollower.set(TalonFXControlMode.Velocity, flywheelOutput);
                break;
            case NONE:
            default:
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, 0.0);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    private double convertToHoodAngle(double pPotentiometerReading) {
        return 90.0 - pPotentiometerReading / kHoodConversionFactor - kMinShotDegrees;
    }


    private double convertFromHoodAngle(double pHoodAngle) {
        return (90.0-pHoodAngle - kMinShotDegrees) * kHoodConversionFactor;
    }

    private double setHood(double pNow) {
        HoodState state = db.flywheel.get(HOOD_STATE, HoodState.class);
        if(state == null) state = HoodState.NONE;

        switch(state) {
            case MANUAL:
                mHoodServo.setServo(db.flywheel.get(HOOD_OPEN_LOOP));
                break;
            case TARGET_ANGLE:
                double target = convertFromHoodAngle(db.flywheel.get(TARGET_HOOD_ANGLE));
                double current = convertFromHoodAngle(db.flywheel.get(CURRENT_HOOD_ANGLE));

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


}