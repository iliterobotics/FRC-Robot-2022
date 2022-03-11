package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.Pigeon;

import static us.ilite.common.types.drive.EDriveData.*;

public class NeoDriveModule extends Module {
    private CANSparkMax mRightMaster;
    private CANSparkMax mRightFollower;
    private CANSparkMax mLeftMaster;
    private CANSparkMax mLeftFollower;
    private RelativeEncoder mRightEncoder;
    private RelativeEncoder mLeftEncoder;
    private SparkMaxPIDController mRightCtrl;
    private SparkMaxPIDController mLeftCtrl;
    private PIDController mTurnToDegreePID;
    private Pigeon mGyro;

    // ========================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS
    // ========================================
    public static final double kGearboxRatio = (12.0 / 40.0) * (14.0 / 40.0);
    public static final double kWheelDiameterFeet = 3.9 / 12.0;
    public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;
    public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;

    // ========================================
    // DO NOT MODIFY THESE PID CONSTANTS
    // ========================================
    private static final int VELOCITY_PID_SLOT = 1;
    private static final int SMART_MOTION_PID_SLOT = 2;
    //TODO change the smart motion gains once we have tuned it
    public static ProfileGains kSmartMotionGains = new ProfileGains()
            .p(.00025)
            .f(0.00015)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .maxAccel(1000d)
            .slot(SMART_MOTION_PID_SLOT)
            .velocityConversion(kDriveNEOPositionFactor);
    public static ProfileGains kVelocityGains = new ProfileGains()
            .f(0.5)
            .p(0.00051968)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .slot(VELOCITY_PID_SLOT)
            .velocityConversion(kDriveNEOVelocityFactor);
    public static ProfileGains kTurnToProfileGains = new ProfileGains().p(0.0285);

    // ========================================
    // DO NOT MODIFY THESE OTHER CONSTANTS
    // ========================================
    private static double mLeftHoldPosition = 0;
    private static double mRightHoldPosition = 0;
    private static int mCyclesHolding = 0;
    private static double kTurnSensitivity = 0.85;
    private static double kInitialXPosition = 0;
    private static double kInitialYPosition = 0;
    private DifferentialDriveOdometry mOdometry;
    private DifferentialDrive mDrive;

    public NeoDriveModule() {
        mLeftMaster = new CANSparkMax(Settings.HW.CAN.kDTML1, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftFollower = new CANSparkMax(Settings.HW.CAN.kDTL3, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMaster = new CANSparkMax(Settings.HW.CAN.kDTMR2, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightFollower = new CANSparkMax(Settings.HW.CAN.kDTR4, CANSparkMaxLowLevel.MotorType.kBrushless);
        mGyro = new Pigeon(Robot.CLOCK, Settings.HW.CAN.kDTGyro);

        mLeftMaster.burnFlash();
        mLeftFollower.burnFlash();
        mRightMaster.burnFlash();
        mRightFollower.burnFlash();
        mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightMaster.setInverted(true);
        mRightFollower.setInverted(true);

        mRightEncoder = mRightMaster.getEncoder();
        mLeftEncoder = mLeftMaster.getEncoder();

        mRightCtrl = mRightMaster.getPIDController();
        mLeftCtrl = mLeftMaster.getPIDController();
        mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);
        mLeftCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);

        mTurnToDegreePID = new PIDController(kTurnToProfileGains, -180, 180, Settings.kControlLoopPeriod);
        mTurnToDegreePID.setContinuous(true);
        mTurnToDegreePID.setOutputRange(-1, 1);

        HardwareUtils.setGains(mLeftCtrl, kVelocityGains);
        HardwareUtils.setGains(mRightCtrl, kVelocityGains);
        HardwareUtils.setGains(mLeftCtrl, kSmartMotionGains);
        HardwareUtils.setGains(mRightCtrl, kSmartMotionGains);

        mOdometry = new DifferentialDriveOdometry(new Rotation2d());
    }
    @Override
    public void readInputs() {
        db.drivetrain.set(ACTUAL_HEADING_RADIANS, -mGyro.getHeading().getRadians());
        db.drivetrain.set(ACTUAL_HEADING_DEGREES, -mGyro.getHeading().getDegrees());
        db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getOutputCurrent());
        db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getOutputCurrent());
        db.drivetrain.set(L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(L_ACTUAL_POS_meters, Units.feet_to_meters(mLeftEncoder.getPosition() * kDriveNEOPositionFactor));
        db.drivetrain.set(L_ACTUAL_VEL_meters_s, Units.feet_to_meters(mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor));
        db.drivetrain.set(R_ACTUAL_POS_meters, Units.feet_to_meters(mRightEncoder.getPosition() * kDriveNEOPositionFactor));
        db.drivetrain.set(R_ACTUAL_VEL_meters_s, Units.feet_to_meters(mRightEncoder.getVelocity() * kDriveNEOVelocityFactor));
        db.imu.set(EGyro.ACCEL_X, mGyro.getAccelX());
        db.imu.set(EGyro.ACCEL_Y, mGyro.getAccelY());
        db.imu.set(EGyro.PITCH_DEGREES, mGyro.getPitch().getDegrees());
        db.imu.set(EGyro.ROLL_DEGREES, mGyro.getRoll().getDegrees());
        db.imu.set(EGyro.YAW_DEGREES, mGyro.getYaw().getDegrees());
        db.imu.set(EGyro.YAW_OMEGA_DEGREES, mGyro.getYawRate().getDegrees());
    }

    @Override
    public void setOutputs() {
        Enums.EDriveState state = db.drivetrain.get(STATE, Enums.EDriveState.class);
        double throttle = db.drivetrain.get(DESIRED_THROTTLE_PCT);
        double turn = db.drivetrain.get(DESIRED_TURN_PCT);
        double left = throttle + turn;
        double right = throttle - turn;
        if (state == null) return;
        switch (state) {
            case RESET:
                reset();
                break;
            case PERCENT_OUTPUT:
                mLeftMaster.set(left);
                mRightMaster.set(right);
                break;
            case VELOCITY:
                mLeftCtrl.setReference(left * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mRightCtrl.setReference(right * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mCyclesHolding = 0;
                break;
            case TURN_TO:
                mTurnToDegreePID.setSetpoint(db.drivetrain.get(DESIRED_TURN_ANGLE_deg));
                double output = mTurnToDegreePID.calculate(db.drivetrain.get(ACTUAL_HEADING_DEGREES), clock.getCurrentTimeInMillis());
                db.drivetrain.set(DESIRED_TURN_PCT, output);
                mLeftMaster.set(output);
                mRightMaster.set(-output);
                break;
            case SMART_MOTION:
                mLeftCtrl.setReference( db.drivetrain.get(L_DESIRED_POS_FT) / kDriveNEOPositionFactor,
                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
                mRightCtrl.setReference( db.drivetrain.get(L_DESIRED_POS_FT) / kDriveNEOPositionFactor,
                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
                break;
        }
    }

    private void reset() {
        mLeftEncoder.setPosition(0.0);
        mRightEncoder.setPosition(0.0);
        mLeftMaster.set(0.0);
        mRightMaster.set(0.0);
    }
}
