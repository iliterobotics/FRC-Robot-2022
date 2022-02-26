package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.ELimelightData.TV;
import static us.ilite.common.types.ELimelightData.TX;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.common.types.sensor.EPowerDistPanel;
import static us.ilite.common.types.sensor.EPowerDistPanel.*;
import us.ilite.robot.Robot;
import static us.ilite.robot.Enums.*;

import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.hardware.*;

/**
 * Class for running all drivetrain train control operations from both autonomous and
 * driver-control.
 */
public class VioletDriveModule extends Module {
    private final ILog mLogger = Logger.createLog(DriveModule.class);

    // DO NOT CHANGE THE GEAR RATIO
    public static double kGearboxRatio = (10.0 / 40.0) * (14.0 / 40.0);

    // As of 2/9this is for omni wheels on a solid floor
    public static double kWheelDiameterInches = 5.875;
    public static double kWheelCircumferenceFeet = kWheelDiameterInches*Math.PI/12.0;
    public static double kWheelbaseDiagonalFeet = 34.853; // TODO Verify whether this is accurate

    // For position, getPosition() returns raw rotations - so convert that to feet
    public static double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;

    // Actual measured was 5514 with a resting battery voltage of 12.75V
    public static double kDriveTrainMaxVelocityRPM = 5500.0;
    public static Distance kDriveMaxVelocity_measured = Distance.fromFeet(kDriveTrainMaxVelocityRPM*kDriveNEOVelocityFactor);
    public static Distance kDriveMaxAccel_simulated = Distance.fromFeet(28.5);
    public static double feetToMeters = 0.3408;

    // This is approx 290 Degrees per second, measured with a Pigeon
    // Actual measured was 825 Degrees per second, with a resting battery voltage of 12.57V
    public static double kMaxDegreesPerSecond = 300;
    // This is with the ADIS16470 IMU
    public static Rotation2d kDriveMaxOmega_measured = Rotation2d.fromDegrees(19.1);

    public static double kEffectiveWheelbase = 23.25;
    public static double kCurvatureCircumference = kEffectiveWheelbase * Math.PI;
    public static double kInchesPerCurvatureDegree = kCurvatureCircumference / 360.0;
    public static double kWheelTurnsPerCurvatureDegree = kInchesPerCurvatureDegree / kWheelDiameterInches;


    public static double kClosedLoopVoltageRampRate = 0.1 ;
    public static double kDefaultRampRate = 120.0; // in V/sec
    public static double kOpenLoopVoltageRampRate = 0.1;
    public static int kCurrentLimitAmps = 50;
    public static int kCurrentLimitTriggerDurationMs = 100;

    // =============================================================================
    // Closed-Loop Velocity Constants
    // =============================================================================

    private static final int VELOCITY_PID_SLOT = 1;
    private static final int POSITION_PID_SLOT = 2;
    private static final int SMART_MOTION_PID_SLOT = 3;

    private int mStartAngleDeg = 0;
    private static DifferentialDriveOdometry mOdometry;
    private static DifferentialDrive mDrive;

    public static ProfileGains kPositionGains = new ProfileGains()
            .p(0.000025)
            .maxVelocity(Settings.Input.kMaxAllowedVelocityMultiplier)
            // Divide by the simulated blue nitrile CoF 1.2, multiply by omni (on school floor) theoretical of 0.4
            .maxAccel(kDriveMaxAccel_simulated.feet() / kDriveNEOVelocityFactor / 1.2 * 0.8)
            .slot(POSITION_PID_SLOT)
            .velocityConversion(kDriveNEOPositionFactor);
    public static ProfileGains smartMotionPID = new ProfileGains()
            .p(.00025)
            .f(0.00015)
            .maxVelocity(kDriveTrainMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .maxAccel(1000d)
            .slot(SMART_MOTION_PID_SLOT)
            .velocityConversion(kDriveNEOPositionFactor);

    public static ProfileGains kVelocityGains = new ProfileGains()
        .f(0.00015)
        .p(0.00000025)
        .maxVelocity(Settings.Input.kMaxAllowedVelocityMultiplier)
        // Divide by the simulated blue nitrile CoF 1.2, multiply by omni (on school floor) theoretical of 0.4
        .maxAccel(kDriveMaxAccel_simulated.feet() / kDriveNEOVelocityFactor / 1.2 * 0.8)
        .slot(VELOCITY_PID_SLOT)
        .velocityConversion(1d);
    public static ProfileGains kPracticeGains = new ProfileGains()
            .p(Settings.kP)
            .maxVelocity(Settings.Input.kMaxAllowedVelocityMultiplier)
            .slot(VELOCITY_PID_SLOT)
            .kS(Settings.kS)
            .kA(Settings.kA)
            .kV(Settings.kV);

    public static ProfileGains vGains = new ProfileGains()
        .p(0.0001)
        .f(0.0001)
        .maxVelocity(Settings.Input.kMaxAllowedVelocityMultiplier)
        .maxAccel(1000);
        ;

    public static ProfileGains kTankGains = new ProfileGains()
        .p(0.00000025)
        .f(0.00015)
        .maxVelocity(Settings.Input.kMaxAllowedVelocityMultiplier)
        ;



    public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);

    public static double kTurnSensitivity = 0.85;

    // =============================================================================
    // Heading Gains
    // =============================================================================

    public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
    public static ProfileGains kYawGains = new ProfileGains().p(0.1);
    public IMU mGyro;
    private double mLastHeading = 0;
    private double mDeltaTime = 0;
    private double mLastTime = 0;

    // =============================================================================
    // Hold Gains
    // =============================================================================

    public static ProfileGains kHoldPositionGains = new ProfileGains().p(.001);//.d(.00807);

    public static EPowerDistPanel[] kPdpSlots = new EPowerDistPanel[]{
            /* Left */
            CURRENT1,
            CURRENT2,

            /* Right */
            CURRENT13,
            CURRENT14,

    };

//    private Rotation2d mGyroOffset = new Rotation2d();
//    private PIDController mYawPid;
    private PIDController mTargetAngleLockPid;
    private PIDController mLeftVelocityPID;
    private PIDController mRightVelocityPID;
    private PIDController mLeftPositionPID;
    private PIDController mRightPositionPID;

    private PIDController mLeftTankPID;
    private PIDController mRightTankPID;

    private ILITEPIDController mVelocityPID;

    private double mLeftHoldSetpoint;
    private double mRightHoldSetpoint;
    private boolean mStartHoldingPosition;
    private double initialXPosition;
    private double initialYPosition;


    private final CANSparkMax mLeftMaster;
    private final CANSparkMax mLeftFollower;
    private final CANSparkMax mRightMaster;
    private final CANSparkMax mRightFollower;

    private final RelativeEncoder mLeftEncoder;
    private final RelativeEncoder mRightEncoder;

    private static final SparkMaxFactory.Configuration kDriveConfig = new SparkMaxFactory.Configuration();
    static {
        kDriveConfig.IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    }

    public VioletDriveModule() {
        mLeftMaster = SparkMaxFactory.createSparkMax(3, kDriveConfig);
        mLeftFollower = SparkMaxFactory.createSparkMax(4, kDriveConfig);
        SmartDashboard.putNumber("Odometry posX", -1);
        SmartDashboard.putNumber("Odometry posY", -1);
        SmartDashboard.putNumber("Odometry init posX", -1);
        SmartDashboard.putNumber("Odometry init posY", -1);

        mLeftFollower.follow(mLeftMaster);
        mLeftEncoder = mLeftMaster.getEncoder();

        mRightMaster = SparkMaxFactory.createSparkMax(1, kDriveConfig);
        mRightFollower = SparkMaxFactory.createSparkMax(2, kDriveConfig);
        mRightFollower.follow(mRightMaster);
        mRightEncoder = mRightMaster.getEncoder();
        mRightMaster.setInverted(true);
        mRightFollower.setInverted(true);

        mGyro = new Pigeon(clock, 21);

        double ramprate = 0.20;
        mLeftMaster.setClosedLoopRampRate(ramprate);
        mLeftFollower.setClosedLoopRampRate(ramprate);
        mRightMaster.setClosedLoopRampRate(ramprate);
        mRightFollower.setClosedLoopRampRate(ramprate);
        mDrive = new DifferentialDrive(mLeftMaster, mRightMaster);
        

        HardwareUtils.setGains(mLeftEncoder, kVelocityGains);
        HardwareUtils.setGains(mRightEncoder, kVelocityGains);
        HardwareUtils.setGains(mLeftEncoder, kPositionGains);
        HardwareUtils.setGains(mRightEncoder, kPositionGains);
        HardwareUtils.setGains(mLeftEncoder, smartMotionPID);
        HardwareUtils.setGains(mRightEncoder, smartMotionPID);

        mLeftVelocityPID = new PIDController( kVelocityGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);
        mRightVelocityPID = new PIDController( kVelocityGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);

        mLeftPositionPID = new PIDController( kPositionGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);
        mRightPositionPID = new PIDController( kPositionGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);

        mLeftTankPID = new PIDController(kTankGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);
        mRightTankPID = new PIDController(kTankGains, -kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM, Settings.kControlLoopPeriod);

        mVelocityPID = new ILITEPIDController(ILITEPIDController.EPIDControlType.VELOCITY, vGains, clock);

        //TODO - we want to do use our conversion factor calculated above, but that requires re-turning of F & P
        mLeftEncoder.setPositionConversionFactor(1d);
        mLeftEncoder.setVelocityConversionFactor(1d);
        mRightEncoder.setPositionConversionFactor(1d);
        mRightEncoder.setVelocityConversionFactor(1d);
        mLeftMaster.burnFlash();
        mLeftFollower.burnFlash();
        mRightMaster.burnFlash();
        mRightFollower.burnFlash();
        mOdometry = new DifferentialDriveOdometry(mGyro.getHeading());
        mDrive = new DifferentialDrive(mLeftMaster, mRightMaster);
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        mGyro.zeroAll();
        mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput, Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);
        mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
        mTargetAngleLockPid.setSetpoint(0);
        mTargetAngleLockPid.reset();
        
        mLeftVelocityPID.setOutputRange(-Settings.Input.kMaxAllowedVelocityMultiplier, Settings.Input.kMaxAllowedVelocityMultiplier);
        mLeftVelocityPID.setSetpoint(0);
        mLeftVelocityPID.reset();
        mLeftVelocityPID.setDeadband(0.2);

        mRightVelocityPID.setOutputRange(-Settings.Input.kMaxAllowedVelocityMultiplier, Settings.Input.kMaxAllowedVelocityMultiplier);
        mRightVelocityPID.setSetpoint(0);
        mRightVelocityPID.reset();
        mRightVelocityPID.setDeadband(0.2);

        mLeftPositionPID.setOutputRange(-Settings.Input.kMaxAllowedVelocityMultiplier, Settings.Input.kMaxAllowedVelocityMultiplier);
        mLeftPositionPID.setSetpoint(0);
        mLeftPositionPID.reset();
        mLeftPositionPID.setDeadband(Settings.Input.kInputDeadbandF310Joystick);

        mRightPositionPID.setOutputRange(-Settings.Input.kMaxAllowedVelocityMultiplier, Settings.Input.kMaxAllowedVelocityMultiplier);
        mRightPositionPID.setSetpoint(0);
        mRightPositionPID.reset();
        mRightPositionPID.setDeadband(Settings.Input.kInputDeadbandF310Joystick);

        mVelocityPID.setInputRange(-kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM);
        mVelocityPID.setOutputRange(-Settings.Input.kMaxAllowedVelocityMultiplier, Settings.Input.kMaxAllowedVelocityMultiplier);

        mStartHoldingPosition = false;
        SmartDashboard.putNumber("Heading", mGyro.getHeading().getDegrees());

        reset();
        System.err.println(" ==== DRIVE MAX ACCEL (RPM): " + (kDriveMaxAccel_simulated.feet() / kDriveNEOVelocityFactor / 1.2 * 0.4));

        mLeftMaster.setClosedLoopRampRate(0.2);
        mLeftFollower.setClosedLoopRampRate(0.2);

        mRightMaster.setClosedLoopRampRate(0.2);
        mRightFollower.setClosedLoopRampRate(0.2);
        resetOdometry(TrajectoryCommandUtils.getJSONTrajectory().getInitialPose());
        initialXPosition = mOdometry.getPoseMeters().getX();
        initialYPosition = mOdometry.getPoseMeters().getY();
        SmartDashboard.putNumber("Odometry init posX", mOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry init posY", mOdometry.getPoseMeters().getY());
    }

    @Override
    public void readInputs() {
        mGyro.update();
        SmartDashboard.putNumber("VioletDrive::Gyro Heading",mGyro.getHeading().getDegrees());
        db.drivetrain.set(DELTA_HEADING, mGyro.getHeading().getDegrees() - mLastHeading);
        db.drivetrain.set(GYRO_RATE, db.drivetrain.get(DELTA_HEADING) / mDeltaTime);
        db.drivetrain.set(L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(L_ACTUAL_VEL_RPM, mLeftEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(R_ACTUAL_VEL_RPM, mRightEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getOutputCurrent());
        db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getOutputCurrent());
        double odoX = mOdometry.getPoseMeters().getX()-initialXPosition;
        double odoY = mOdometry.getPoseMeters().getY()-initialYPosition;
        db.drivetrain.set(GET_X_OFFSET_METERS, odoX);
        db.drivetrain.set(GET_Y_OFFSET_METERS, odoY);
        SmartDashboard.putNumber("Odometry posX", odoX);
        SmartDashboard.putNumber("Odometry posY", odoY);

        db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(ACTUAL_HEADING_RADIANS, mGyro.getHeading().getRadians());
        db.drivetrain.set(IS_CURRENT_LIMITING, EPowerDistPanel.isAboveCurrentThreshold(kCurrentLimitAmps, Robot.DATA.pdp, kPdpSlots));
        db.drivetrain.set(ACTUAL_TURN_ANGLE_deg, mGyro.getHeading().getDegrees());
        db.imu.set(EGyro.HEADING_DEGREES, -mGyro.getHeading().getDegrees());
        db.imu.set(EGyro.YAW_OMEGA_DEGREES, mGyro.getYawRate().getDegrees());

        mOdometry.update(
                mGyro.getHeading(), Units.feetToMeters(mLeftEncoder.getPosition() * kDriveNEOPositionFactor),
                Units.feetToMeters(mRightEncoder.getPosition() * kDriveNEOPositionFactor));

        Robot.mField.setRobotPose(mOdometry.getPoseMeters());
    }

    @Override
    public void setOutputs() {
        mLastHeading = mGyro.getHeading().getDegrees();
        EDriveState mode = db.drivetrain.get(STATE, EDriveState.class);
        // Do this to prevent wonkiness while transitioning autonomous to teleop
        if(mode == null) return;
        double turn = db.drivetrain.safeGet(DESIRED_TURN_PCT, 0.0);
        double throttle = db.drivetrain.safeGet(DESIRED_THROTTLE_PCT, 0.0);

        double left = db.drivetrain.safeGet(DESIRED_LEFT_PCT, 0d);
        double right = db.drivetrain.safeGet(DESIRED_RIGHT_PCT, 0d);

        switch (mode) {
            case RESET:
                reset();
                break;
            case HOLD:
                if (!mStartHoldingPosition) {
                    mLeftHoldSetpoint = db.drivetrain.get(L_ACTUAL_POS_FT);
                    mRightHoldSetpoint = db.drivetrain.get(R_ACTUAL_POS_FT);
                    mStartHoldingPosition = true;
                }

                double holdLeft = 0;
                double holdRight = 0;

                mLeftVelocityPID.setSetpoint(mLeftHoldSetpoint);
                mRightVelocityPID.setSetpoint(mRightHoldSetpoint);

                if (mLeftEncoder.getVelocity() < 100) {
                    if (Math.abs(mLeftEncoder.getPosition() - mLeftHoldSetpoint) > .15) {
                        holdLeft = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                    }
                } else {
                    holdLeft = 0;
                }

                if (mRightEncoder.getVelocity() < 100) {
                    if (Math.abs(mRightEncoder.getPosition() - mRightHoldSetpoint) > .15) {
                        holdRight = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                    }
                } else {
                    holdRight = 0;
                }

                mLeftMaster.set(holdLeft);
                mRightMaster.set(holdRight);
                break;
            case TARGET_ANGLE_LOCK:
//                double pidOutput = 0;
                if(mTargetAngleLockPid != null && db.limelight != null && db.limelight.isSet(TV) && db.limelight.isSet(TX)) {
//                    //if there is a target in the limelight's fov, lock onto target using feedback loop
//                    pidOutput = mTargetAngleLockPid.calculate(-1.0 * db.limelight.get(TX), clock.dt());
//                    pidOutput = pidOutput + (Math.signum(pidOutput) * Settings.kTargetAngleLockFrictionFeedforward);
//					SmartDashboard.putNumber("Target Angle Lock PID Output", pidOutput);
//                    turn = pidOutput;
                }

//                mLeftMaster.set(pidOutput);
//                mRightMaster.set(-pidOutput);
//                mLeftMaster.set(pidOutput);
//                mRightMaster.set(-pidOutput);
                // NOTE - fall through here
            case VELOCITY:
                mStartHoldingPosition = false;

                mLeftVelocityPID.setSetpoint((throttle+turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);
                mRightVelocityPID.setSetpoint((throttle-turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);

                mLeftVelocityPID.setSetpoint((throttle+turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);
                mRightVelocityPID.setSetpoint((throttle-turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);
                double vLeft = mLeftVelocityPID.calculate(db.drivetrain.get(L_ACTUAL_VEL_RPM), clock.getCurrentTimeInMillis());
                double vRight = mRightVelocityPID.calculate(db.drivetrain.get(R_ACTUAL_VEL_RPM), clock.getCurrentTimeInMillis());

//                double vLeft = mLeftVelocityPID.calculate(db.drivetrain.get(L_ACTUAL_VEL_RPM),
//                                                            (throttle+turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);
//                double vRight = mLeftVelocityPID.calculate(db.drivetrain.get(R_ACTUAL_VEL_RPM),
//                        (throttle-turn) * kDriveTrainMaxVelocityRPM* Settings.Input.kMaxAllowedVelocityMultiplier);

                mLeftMaster.set(vLeft);
                mRightMaster.set(vRight);
                break;
            case PERCENT_OUTPUT:
                mLeftMaster.set((throttle+turn)*Settings.Input.kMaxAllowedVelocityMultiplier);
                mRightMaster.set((throttle-turn)*Settings.Input.kMaxAllowedVelocityMultiplier);
                break;
            case PATH_FOLLOWING_BASIC:
                mLeftPositionPID.setSetpoint(db.drivetrain.get(L_DESIRED_POS_FT));
                mRightPositionPID.setSetpoint(db.drivetrain.get(R_DESIRED_POS_FT));

                double posLeft = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                double posRight = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());

                mLeftMaster.set(posLeft);
                mRightMaster.set(posRight);
                break;
            case PATH_FOLLOWING_RAMSETE:
//                mLeftMaster.setVoltage(db.drivetrain.get(DESIRED_LEFT_VOLTAGE));
//                mRightMaster.setVoltage(db.drivetrain.get(DESIRED_RIGHT_VOLTAGE));
//                mDrive.feed();
                mLeftMaster.set(db.drivetrain.get(L_DESIRED_DRIVE_FT_SEC)/24);
                mRightMaster.set(db.drivetrain.get(R_DESIRED_DRIVE_FT_SEC)/24);
                mDrive.feed();
                break;
            case TURN_FOR:
                double arcLengthFor = kWheelbaseDiagonalFeet * kGearboxRatio * Math.PI * db.drivetrain.get(DESIRED_TURN_ANGLE_deg) / 360.0;

                mLeftPositionPID.setSetpoint(arcLengthFor);
                mRightPositionPID.setSetpoint(-arcLengthFor);

                double leftOutputFor = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                double rightOutputFor = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());

                mLeftMaster.set(leftOutputFor);
                mRightMaster.set(rightOutputFor);
                break;
            case TURN_TO:
                double arcLengthTo = kWheelbaseDiagonalFeet * kGearboxRatio * Math.PI * (db.drivetrain.get(DESIRED_TURN_ANGLE_deg) - mStartAngleDeg) / 360.0;

                mLeftPositionPID.setSetpoint(arcLengthTo);
                mRightPositionPID.setSetpoint(-arcLengthTo);

                double leftOutputTo = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                double rightOutputTo = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());

                mLeftMaster.set(leftOutputTo);
                mRightMaster.set(rightOutputTo);
                break;
            case HOME:
                double arcLengthHome = kWheelbaseDiagonalFeet * kGearboxRatio * Math.PI * -mStartAngleDeg / 360.0;

                mLeftPositionPID.setSetpoint(arcLengthHome);
                mLeftPositionPID.setSetpoint(-arcLengthHome);

                double leftOutputHome = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
                double rightOutputHome = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());

                mLeftMaster.set(leftOutputHome);
                mRightMaster.set(rightOutputHome);
                break;
            case TANK:
                mLeftTankPID.setSetpoint((left)*kDriveTrainMaxVelocityRPM*Settings.Input.kMaxAllowedVelocityMultiplier);
                mRightTankPID.setSetpoint((right)*kDriveTrainMaxVelocityRPM*Settings.Input.kMaxAllowedVelocityMultiplier);

                double vLeftTank = mLeftTankPID.calculate(db.drivetrain.get(L_ACTUAL_VEL_RPM), clock.getCurrentTimeInMillis());
                double vRightTank = mRightTankPID.calculate(db.drivetrain.get(R_ACTUAL_VEL_RPM), clock.getCurrentTimeInMillis());

                mLeftMaster.set(vLeftTank);
                mRightMaster.set(vRightTank);
                break;
        }
    }
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public static Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        mOdometry.resetPosition(pose, mGyro.getHeading());
    }

    private void reset() {
        mLeftEncoder.setPosition(0.0);
        mRightEncoder.setPosition(0.0);
    }

}