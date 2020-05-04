package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.EMatchMode;
import static us.ilite.robot.Enums.*;

import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EShooterSystemData.*;


public class FlywheelModule extends Module {

    // Flywheel Motors
    private CANSparkMax mTurret;
    private TalonFX mFlywheelFalconMaster;
    private TalonFX mFlywheelFalconFollower;
    private CANSparkMax mFeeder;
    private ContinuousRotationServo mHoodServo;

    // These were for direct RIO connections
//    private AnalogInput mHoodAI;
//    private AnalogPotentiometer mHoodPot;

    // This is for the feeder MAX connection
    private CANAnalog mHoodPot;

    // Constants
    private static final double kHoodGearRatio = 32.0 / 20.0 * 14.0 / 360.0;//First stage was 32/20 in code but 36/20 in integration
    public static final double kTurretGearRatio = 9.0 / 72.0 * 18.0 / 160.0;
    private static final double kFlyWheelGearRatio = 36.0 / 24.0;
    private static final double kFlywheelDiameterInches = 4.0;
    private static double kFlywheelRadiusFeet = kFlywheelDiameterInches / 2.0 / 12.0;


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
    private static final double kMaxTurretPercentOutput = 0.8;
    private static final double kMaximumTurretAngle = 45.0;
    private static final double kMaxTurretAngleLeft = -45.0;
    private static final double kMaxTurretAngleRight = 190.0;
    private final double kTurretErrorTolerance = 5.0; //TODO - tune tolerance
    private final double kTurretFront = 0.0;
    private final double kTurretBack = 180.0;

    private static double kRadiansPerSecToTalonTicksPer100ms = (2 * Math.PI) / 2048.0 / 0.1;
    // https://docs.google.com/spreadsheets/d/1Po6exzGvfr0rMWMWVSyqxf49ZMSfGoYn/edit#gid=1858991275
    // Converts from desired BALL (not wheel) velocity to Falcon ticks per 100ms
    private static double kMOISlipFactor = 1.25;
    private static double kVelocityConversion = kMOISlipFactor * (2048.0 / 600.0) / (kFlyWheelGearRatio * kFlywheelDiameterInches * Math.PI / 12.0 / 60.0) * 2.0;

    private static final int FLYWHEEL_SLOT = 0;
    private static final int TURRET_SLOT = 0;
    private static ProfileGains kFlywheelGains = new ProfileGains()
            .slot(FLYWHEEL_SLOT)
            .p(0.05)
            .f(0.0575)
//            .kA(0.00165)
//            .kV(0.018)
            ;
    private static final double kTurretPReduction = 0.8;
    // Smart motion turret gains
    private static ProfileGains kTurretCtrlGains = new ProfileGains()
            .p(.0002 * kTurretPReduction)
            .maxVelocity(1000d)
            .maxAccel(1000d)
            .slot(TURRET_SLOT);
//     Percent output turret gains
    private static ProfileGains kTurretGains = new ProfileGains()
            .p(.02);

    private PIDController mHoodPID = new PIDController(5, 0, 0);

    private CANEncoder mFeederInternalEncoder;

    private CANPIDController mTurretCtrlPID;
    private us.ilite.common.lib.control.PIDController mTurretPID = new us.ilite.common.lib.control.PIDController(kTurretGains, -90, 90, Settings.kControlLoopPeriod);

    private CANEncoder mTurretEncoder;


    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        mTurret = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kSRXTurretId, CANSparkMaxLowLevel.MotorType.kBrushless);

        mTurretEncoder = mTurret.getEncoder();
        mTurretPID.setOutputRange(-kMaxTurretPercentOutput, kMaxTurretPercentOutput);
        mTurretCtrlPID = mTurret.getPIDController();
        mTurret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mTurret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mTurret.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,  (float) Units.degrees_to_rotations(kMaxTurretAngleRight, kTurretGearRatio));
        mTurret.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Units.degrees_to_rotations(kMaxTurretAngleLeft, kTurretGearRatio));
        mTurretEncoder.setPosition(0.0);
        HardwareUtils.setGains(mTurretCtrlPID, kTurretCtrlGains);

//        mHoodPot = new AnalogPotentiometer(0);
//        SmartDashboard.putNumber("kRadiansPerSecToTalonTicksPer100ms", kRadiansPerSecToTalonTicksPer100ms);
//        SmartDashboard.putNumber("kVelocityConversion", kVelocityConversion);
        mFlywheelFalconMaster = new TalonFX(Settings.HW.CAN.kFalconMasterId);
        mFlywheelFalconMaster.setInverted(true);
        mFlywheelFalconMaster.setNeutralMode(NeutralMode.Coast);
        mFlywheelFalconMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        mFlywheelFalconFollower = new TalonFX(Settings.HW.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Coast);
        mFlywheelFalconFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kMAXFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);
        mFeederInternalEncoder = new CANEncoder(mFeeder);
        mFeeder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mFeeder.setSmartCurrentLimit(30);

        mHoodServo = new ContinuousRotationServo(Settings.HW.PWM.kHoodServoId).inverted(true);
        mHoodPot = mFeeder.getAnalog(CANAnalog.AnalogMode.kAbsolute);
//        mHoodAI = new AnalogInput(Settings.Hardware.Analog.kHoodPot);
//        mHoodPot = new AnalogPotentiometer(mHoodAI);
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        //Set PID gains & zero encoders here
        HardwareUtils.setGains(mFlywheelFalconMaster, kFlywheelGains);
        HardwareUtils.setGains(mFlywheelFalconFollower, kFlywheelGains);
    }

    @Override
    public void readInputs() {
        double raw = mFlywheelFalconMaster.getSelectedSensorVelocity();
        db.flywheel.set(BALL_VELOCITY_ft_s, raw / kVelocityConversion);
        db.flywheel.set(FLYWHEEL_RAW_SPEED, raw);
        db.flywheel.set(FEEDER_rpm, mFeederInternalEncoder.getVelocity());
        double position = mHoodPot.getPosition();
        db.flywheel.set(POT_RAW_VALUE, position);
        db.flywheel.set(HOOD_ANGLE_deg, convertToHoodAngle(position));
        db.flywheel.set(HOOD_SERVO_RAW_VALUE, mHoodServo.getRawOutputValue());
        db.flywheel.set(TURRET_ANGLE_deg, getCurrentTurretAngle());
        //TODO - undo this once the turret is physically fixed
        db.flywheel.set(IS_TARGET_LOCKED, true);//Math.abs(mTurretPID.getError()) <= kTurretErrorTolerance);

//        mPotentiometerReadings.addNumber(mHoodAI.getValue());
//
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
////
//        if (db.goaltracking.isSet(ELimelightData.TV)) {
////            db.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
////            db.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
//        } else {
//            db.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
////            db.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, 0);
//        }
    }

    @Override
    public void setOutputs() {
        setHood();
        setTurret();
        setFlywheel();
        setFeeder();
    }

    @Override
    public void shutdown() {
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_POSITION, 0);
    }

    private void setTurret() {
        if (db.flywheel.isSet(TURRET_CONTROL)) {
            TurretControlType turretControlType = db.flywheel.get(TURRET_CONTROL, TurretControlType.class);

            switch (turretControlType) {
                case MANUAL:
                    double mTurretDirection = db.flywheel.get(MANUAL_TURRET_DIRECTION);
                    if (db.flywheel.isSet(MANUAL_TURRET_DIRECTION)) {
//                        mTurret.set(.25 * mTurretDirection);
                        mTurretCtrlPID.setReference(0.0, ControlType.kPosition, TURRET_SLOT, 0);
                    } else {
                        mTurretCtrlPID.setReference(0.0, ControlType.kPosition, TURRET_SLOT, 0);
                    }
                    break;
                case TARGET_LOCKING:
                    if (db.goaltracking.isSet(EVisionGoal2020.TX)) {
                        mTurretPID.setSetpoint(0.0);
                        double output = -mTurretPID.calculate(db.goaltracking.get(EVisionGoal2020.TX), clock.now());
                        mTurret.set(output);
//                        mTurretPID.setReference(Units.degrees_to_rotations(getCurrentTurretAngle() + db.goaltracking.get(ELimelightData.TX), kTurretGearRatio), ControlType.kSmartMotion, TURRET_SLOT, 0);
                    } else {
                        mTurretCtrlPID.setReference(0.0, ControlType.kPosition, TURRET_SLOT, 0);
                    }
                    break;
                case HOME:
                    db.goaltracking.set(EVisionGoal2020.TARGET_ID, Limelight.NONE.id());
//                    mTurretCtrlPID.setReference(0.0, ControlType.kPosition, TURRET_SLOT, 0);
                    boolean reversed = db.flywheel.isSet(HOME_REVERSED);
                    double turretHome = kTurretFront;
                    if (reversed) {
                        turretHome = kTurretBack;
                    }
                    mTurretPID.setSetpoint(turretHome);
                    double output = mTurretPID.calculate(getCurrentTurretAngle(), clock.now());
                    mTurret.set(output);
                    break;
            }
        }
    }

    private void setFeeder() {
        // Cannot set voltage mode if an external sensor is attached
//        mFeeder.setVoltage(db.flywheel.get(FEEDER_OUTPUT_OPEN_LOOP) / 12.0);
        mFeeder.set(db.flywheel.get(SET_FEEDER_rpm));
    }

    private void setFlywheel() {
        FlywheelWheelState state = db.flywheel.get(FLYWHEEL_WHEEL_STATE, FlywheelWheelState.class);
        if (state == null) state = FlywheelWheelState.NONE;
        switch (state) {
            case OPEN_LOOP:
                double l = db.flywheel.get(FLYWHEEL_OPEN_LOOP);
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, l);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, l);
                break;
            case VELOCITY:
                double flywheelOutput = db.flywheel.get(SET_BALL_VELOCITY_ft_s) * kVelocityConversion;
                mFlywheelFalconMaster.set(TalonFXControlMode.Velocity, flywheelOutput);
                mFlywheelFalconFollower.set(TalonFXControlMode.Velocity, flywheelOutput);
                break;
            case NONE:
            default:
                mFlywheelFalconMaster.set(TalonFXControlMode.PercentOutput, 0.0);
                mFlywheelFalconFollower.set(TalonFXControlMode.PercentOutput, 0.0);
        }
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_POSITION, getCurrentTurretAngle());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_POTENTIOMETER_TURNS, mHoodPot.get());
    }

    private double getCurrentTurretAngle() {
        return Units.rotations_to_degrees(mTurretEncoder.getPosition(), kTurretGearRatio);
    }

    private double convertToHoodAngle(double pPotentiometerReading) {
        return 90.0 - pPotentiometerReading / kHoodConversionFactor - kMinShotDegrees;
    }


    private double convertFromHoodAngle(double pHoodAngle) {
        return (90.0 - pHoodAngle - kMinShotDegrees) * kHoodConversionFactor;
    }

    private void setHood() {
        HoodState state = db.flywheel.get(HOOD_STATE, HoodState.class);
        if (state == null) state = HoodState.NONE;

        switch (state) {
            case MANUAL:
                mHoodServo.setServo(db.flywheel.get(SET_HOOD_pct));
                break;
            case TARGET_ANGLE:
                double target = convertFromHoodAngle(db.flywheel.get(SET_HOOD_ANGLE_deg));
                double current = convertFromHoodAngle(db.flywheel.get(HOOD_ANGLE_deg));

                double output = mHoodPID.calculate(current, target);
                mHoodServo.setServo(output);
                break;
            case NONE:
            default:
                mHoodServo.setServo(0.0);
        }
    }

//    private double calcSpeedFromDistance(Distance pDistance) {
//        //TODO figure out necessity
//        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
//                - 0.209 * Math.pow(pDistance.inches(), 2)
//                + 6.31 * pDistance.inches()
//                + 227;
//    }

//    private double calcAngleFromDistance(Distance pDistance) {
//        //TODO tuning
//        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
//                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
//                + 0.157 * Math.pow(pDistance.inches(), 2)
//                + 2.94 * pDistance.inches()
//                + 68.2;
//    }
}