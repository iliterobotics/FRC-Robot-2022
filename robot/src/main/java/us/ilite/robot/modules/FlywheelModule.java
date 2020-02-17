package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import static us.ilite.common.types.EShooterSystemData.*;

public class FlywheelModule extends Module {

    public enum EHoodState {
        NONE,
        TARGET_ANGLE,
        MANUAL
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


    // Production bot 0.00175 min, 0.74664 max (normalized from AnalogPotentiometer)
    private static final double kHoodReadingRange = 0.74664 - 0.00175;
    private static final double kMinShotDegrees = 20.0;
    private static final double kMaxShotDegrees = 80.0;
    private static final double kHoodAngleRange = kMaxShotDegrees - kMinShotDegrees;
    private static final double kHoodConversionFactor = kHoodReadingRange / kHoodAngleRange;

    private static final int FLYWHEEL_SLOT = 1;
    private static ProfileGains mFlywheelGains = new ProfileGains()
            .slot(FLYWHEEL_SLOT)
            .p(0.0005)
            ;


    private PIDController mHoodPID = new PIDController(5, 0, 0);


    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
//        mFlywheelFalconMaster = new TalonFX(50);     //programming bot flywheel id
        mFlywheelFalconMaster = new TalonFX(Settings.Hardware.CAN.kFalconMasterId);
        mFlywheelFalconMaster.setNeutralMode(NeutralMode.Brake);

        mFlywheelFalconFollower = new TalonFX(Settings.Hardware.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Brake);
        mFlywheelFalconFollower.setInverted(true);

        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);

        mTurret = TalonSRXFactory.createDefaultTalon(Settings.Hardware.CAN.kSRXTurretId);

        mHoodServo = new ContinuousRotationServo(Settings.Hardware.PWM.kHoodServoId).inverted(true);

        mHoodAI = new AnalogInput(Settings.Hardware.Analog.kHoodPot);
        mHoodPot = new AnalogPotentiometer(mHoodAI);

    }
    
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        //Set PID gains & zero encoders here
    }

    @Override
    public void readInputs(double pNow) {
        db.flywheel.set(CURRENT_FLYWHEEL_VELOCITY, mFlywheelFalconMaster.getSelectedSensorVelocity()); //TODO needs to be converted from ticks per 100ms to RPS
        db.flywheel.set(CURRENT_FEEDER_VELOCITY, mFeeder.get());
        db.flywheel.set(CURRENT_HOOD_ANGLE, mHoodPot.get() / kHoodReadingRange * kHoodAngleRange + kMinShotDegrees);
        db.flywheel.set(POT_NORM_VALUE, mHoodPot.get());
        db.flywheel.set(POT_RAW_VALUE, mHoodAI.getValue());
        db.flywheel.set(HOOD_SERVO_VALUE, mHoodServo.getValue());


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

//        setTurret();
        setHood(pNow);

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

    private double setHood(double pNow) {
        EHoodState state = db.flywheel.get(HOOD_STATE, EHoodState.class);
        switch(state) {
            case NONE:
            case MANUAL:
                mHoodServo.setServo(db.flywheel.get(FLYWHEEL_TEST));
                break;
            case TARGET_ANGLE:
                double target = db.flywheel.get(TARGET_HOOD_ANGLE)* kHoodConversionFactor;
                double current = db.flywheel.get(CURRENT_HOOD_ANGLE)* kHoodConversionFactor;

                double output = mHoodPID.calculate(current, target);
                mHoodServo.setServo(output);
                break;
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