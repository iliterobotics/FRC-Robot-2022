package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import static com.revrobotics.ControlType.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.common.types.sensor.EPowerDistPanel;
import static us.ilite.common.types.sensor.EPowerDistPanel.*;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.*;

/**
 * Class for running all drivetrain train control operations from both autonomous and
 * driver-control.
 * TODO Support for rotation trajectories
 * TODO Turn-to-heading with Motion Magic
 */
public class DriveModule extends Module {
	private final ILog mLogger = Logger.createLog(DriveModule.class);
	public static double kDriveTrainMaxVelocity = 5676;

	// TODO Find out what units this is in
	public static double kMaxHeadingChange = 5;

	public static double kGearboxRatio = (12.0 / 80.0) * (42.0 / 80.0);
	public static double kClosedLoopVoltageRampRate = 0.1 ;
	public static double kOpenLoopVoltageRampRate = 0.1;
	public static int kCurrentLimitAmps = 50;
	public static int kCurrentLimitTriggerDurationMs = 100;
	public static double kWheelDiameterInches = 6.0;
	public static double kWheelDiameterFeet = kWheelDiameterInches / 12.0;
	public static double kWheelCircumference = kWheelDiameterInches * Math.PI;
	public static double kDefaultRampRate = 120.0; // in V/sec
	public static double kTicksPerRotation = 1.0;
	public static double kEffectiveWheelbase = 23.25;
	public static double kTurnCircumference = kEffectiveWheelbase * Math.PI;
	public static double kInchesPerDegree = kTurnCircumference / 360.0;
	public static double kWheelTurnsPerDegree = kInchesPerDegree / kWheelDiameterInches;

	// =============================================================================
	// Closed-Loop Velocity Constants
	// =============================================================================
	private static final int VELOCITY_PID_SLOT = 1;
	private static final int POSITION_PID_SLOT = 2;
	public static ProfileGains dPID = new ProfileGains().p(1.0).maxVelocity(5676d).maxAccel(56760d).slot(POSITION_PID_SLOT);
	public static ProfileGains vPID = new ProfileGains().p(1.5234375e-4).d(0.001174257 * 4).maxVelocity(5676d).maxAccel(56760d).slot(VELOCITY_PID_SLOT);
	public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);
	public static double kTurnSensitivity = 0.85;

	// =============================================================================
	// Heading Gains
	// =============================================================================
	public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
	public static ProfileGains kYawGains = new ProfileGains().f(.15);
	public static double kDriveLinearPercentOutputLimit = 0.5;

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

	private Rotation2d mGyroOffset = new Rotation2d();
	private EDriveState mDriveState;

	private PIDController mTargetAngleLockPid;
	private PIDController mYawPid;
	private PIDController mHoldLeftPositionPid;
	private PIDController mHoldRightPositionPid;
	private boolean mStartHoldingPosition;
	private double mPreviousHeading = 0.0;
	private double mPreviousTime = 0;

	private final CANSparkMax mLeftMaster;
	private final CANSparkMax mLeftFollower;
	private final CANSparkMax mRightMaster;
	private final CANSparkMax mRightFollower;
	private final CANEncoder mLeftEncoder;
	private final CANEncoder mRightEncoder;
	private final CANPIDController mLeftCtrl;
	private final CANPIDController mRightCtrl;

	public DriveModule() {
		mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kDriveLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
		mLeftFollower = SparkMaxFactory.createFollowerSparkMax(Settings.Hardware.CAN.kDriveLeftFollower, mLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
		mLeftEncoder = mLeftMaster.getEncoder();
		mLeftCtrl = mLeftMaster.getPIDController();
		mRightMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kDriveRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
		mRightFollower = SparkMaxFactory.createFollowerSparkMax(Settings.Hardware.CAN.kDriveRightFollower, mRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
		mRightMaster.setInverted(true);
		mRightFollower.setInverted(true);
		mRightEncoder = mLeftMaster.getEncoder();
		mRightCtrl = mRightMaster.getPIDController();
	}

	@Override
	public void modeInit(EMatchMode pMode, double pNow) {
		mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput, Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
		mTargetAngleLockPid.setSetpoint(0);
		mTargetAngleLockPid.reset();

		mYawPid = new PIDController(kYawGains,
									-kMaxHeadingChange,
									kMaxHeadingChange,
									Settings.kControlLoopPeriod);
		mYawPid.setOutputRange(-1, 1);

		mHoldLeftPositionPid = new PIDController(kHoldPositionGains,-99999, 99999, Settings.kControlLoopPeriod);
		mHoldLeftPositionPid.setOutputRange(-1, 1);
		mHoldLeftPositionPid.setSetpoint(0.0);
		mHoldRightPositionPid = new PIDController(kHoldPositionGains,-99999, 99999, Settings.kControlLoopPeriod);
		mHoldRightPositionPid.setOutputRange(-1, 1);
		mHoldRightPositionPid.setSetpoint(0.0);
		mStartHoldingPosition = false;
		mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
		mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);

		mLeftMaster.getEncoder().setPosition(0.0);
		mRightMaster.getEncoder().setPosition(0.0);
		setPIDGains(mLeftCtrl, vPID);
		setPIDGains(mRightCtrl, vPID);
		setPIDGains(mLeftCtrl, dPID);
		setPIDGains(mRightCtrl, dPID);
	  	db.drivetrain.set(EDriveData.DESIRED_STATE, EDriveState.NORMAL);
	  	db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
	  	db.drivetrain.set(DESIRED_TURN_PCT, 0.0);
	}

	private void setPIDGains(CANPIDController pCtrl, ProfileGains pGains) {
		pCtrl.setFF(pGains.F, pGains.PROFILE_SLOT);
		pCtrl.setP(pGains.P, pGains.PROFILE_SLOT);
		pCtrl.setI(pGains.I, pGains.PROFILE_SLOT);
		pCtrl.setD(pGains.D, pGains.PROFILE_SLOT);
		pCtrl.setSmartMotionMaxVelocity(pGains.MAX_VELOCITY, pGains.PROFILE_SLOT);
		pCtrl.setSmartMotionMaxAccel(pGains.MAX_ACCEL, pGains.PROFILE_SLOT);
	}

	@Override
	public void readInputs(double pNow) {
		db.drivetrain.set(LEFT_POS_INCHES, Conversions.ticksToInches(mLeftEncoder.getPosition()));
		db.drivetrain.set(LEFT_VEL_IPS, Conversions.ticksToInches(mLeftEncoder.getVelocity()));
		db.drivetrain.set(LEFT_VEL_TICKS, mLeftEncoder.getVelocity());
		db.drivetrain.set(RIGHT_POS_INCHES, Conversions.ticksToInches(mRightEncoder.getPosition()));
		db.drivetrain.set(RIGHT_VEL_IPS, Conversions.ticksToInches(mRightEncoder.getVelocity()));
		db.drivetrain.set(RIGHT_VEL_TICKS, mRightEncoder.getVelocity());
		db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getOutputCurrent());
		db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getOutputCurrent());
		db.drivetrain.set(IS_CURRENT_LIMITING, EPowerDistPanel.isAboveCurrentThreshold(kCurrentLimitAmps, Robot.DATA.pdp, kPdpSlots));

//		Robot.DATA.imu.set(EGyro.HEADING_DEGREES, mDriveHardware.getImu().getHeading().getDegrees());

//		mCurrentHeading = Robot.DATA.imu.get(EGyro.HEADING_DEGREES);
//		Robot.DATA.imu.set(EGyro.YAW_DEGREES, mCurrentHeading - mPreviousHeading);


	}

	@Override
	public void setOutputs(double pNow) {
		EDriveState mode = db.drivetrain.get(DESIRED_STATE, EDriveState.class);
		switch (mode) {
			case HOLD:
				if (!mStartHoldingPosition) {
					mHoldLeftPositionPid.setSetpoint(db.drivetrain.get(LEFT_POS_INCHES));
					mHoldRightPositionPid.setSetpoint(db.drivetrain.get(RIGHT_POS_INCHES));
					mStartHoldingPosition = true;
				}
				if (Math.abs(db.drivetrain.get(LEFT_POS_INCHES) - mHoldLeftPositionPid.getSetpoint()) > 1) {
					double leftOutput = mHoldLeftPositionPid.calculate(db.drivetrain.get(LEFT_POS_INCHES), pNow);
					mLeftCtrl.setReference(leftOutput * kDriveTrainMaxVelocity, kVelocity, VELOCITY_PID_SLOT, 0);
				}
				if (Math.abs(db.drivetrain.get(RIGHT_POS_INCHES) - mHoldRightPositionPid.getSetpoint()) > 1) {
					double rightOutput = mHoldRightPositionPid.calculate(db.drivetrain.get( RIGHT_POS_INCHES), pNow);
					mRightCtrl.setReference(rightOutput * kDriveTrainMaxVelocity, kVelocity, VELOCITY_PID_SLOT, 0);
				}
				break;

			case VELOCITY:
				mStartHoldingPosition = false;
				mYawPid.setSetpoint(db.drivetrain.get(DESIRED_TURN_PCT) * kMaxHeadingChange);
				double mTurn = mYawPid.calculate(Robot.DATA.imu.get(EGyro.YAW_DEGREES), pNow);
				double mThrottle = db.drivetrain.get(DESIRED_THROTTLE_PCT);
				DriveMessage d = new DriveMessage().turn(mTurn).throttle(mThrottle).normalize();
				SmartDashboard.putNumber("DESIRED YAW", mYawPid.getSetpoint());
				SmartDashboard.putNumber("ACTUAL YAW", (Robot.DATA.imu.get(EGyro.YAW_DEGREES)));
//				mLeftCtrl.setReference(100, kVelocity, VELOCITY_PID_SLOT, 0);//d.getLeftOutput() * kDriveTrainMaxVelocity, kVelocity, VELOCITY_PID_SLOT, 0);
//				mRightCtrl.setReference(100, kVelocity, VELOCITY_PID_SLOT, 0);//d.getRightOutput() * kDriveTrainMaxVelocity, kVelocity, VELOCITY_PID_SLOT, 0);
				mLeftMaster.set(.2);
				mRightMaster.set(.2);
				break;
			case PERCENT_OUTPUT:
				break;
		}
		mPreviousTime = pNow;
		mPreviousHeading = db.imu.get(EGyro.HEADING_DEGREES);
	}

	public void loop(double pNow) {
//		mUpdateTimer.start();
		mDriveState = db.drivetrain.get(DESIRED_STATE, EDriveState.class);
//		switch(mDriveState) {
//			case PATH_FOLLOWING:
//				mDriveHardware.configureMode(ECommonControlMode.VELOCITY);
//			case TARGET_ANGLE_LOCK:
//				mDriveHardware.configureMode(ECommonControlMode.PERCENT_OUTPUT);
//				mDriveHardware.set(DriveMessage.kNeutral);
//				RobotCodex<ELimelightData> targetData = Robot.DATA.limelight;
//				double pidOutput;
//				if(mTargetAngleLockPid != null && targetData != null && targetData.isSet(TV) && targetData.isSet(TX)) {
//
//					//if there is a target in the limelight's fov, lock onto target using feedback loop
//					pidOutput = mTargetAngleLockPid.calculate(-1.0 * targetData.get(TX), pNow - mPreviousTime);
//					pidOutput = pidOutput + (Math.signum(pidOutput) * Settings.kTargetAngleLockFrictionFeedforward);
//
//					double mTargetTrackingThrottle = db.drivetrain.get(TARGET_TRACKING_THROTTLE);
//					mDriveMessage = new DriveMessage().throttle(mTargetTrackingThrottle).turn(pidOutput).calculateCurvature();
//					// If we've already seen the target and lose tracking, exit.
//				}
//				break;
//			case NORMAL:
//				break;
//			default:
//				mLogger.warn("Got drivetrain state: " + mDriveState+" which is unhandled");
//				break;
//		}
//		mPreviousTime = pNow;
//		mUpdateTimer.stop();
	}

	public static class Conversions {

		public static double rotationsToInches(double rotations) {
			return rotations * (kWheelDiameterInches * Math.PI);
		}

		public static double rpmToInchesPerSecond(double rpm) {
			return rotationsToInches(rpm) / 60;
		}

		public static double inchesToRotations(double inches) {
			return inches / kWheelCircumference;
		}

		public static double inchesPerSecondToRpm(double inches_per_second) {
			return inchesToRotations(inches_per_second) * 60;
		}

		public static double radiansPerSecondToTicksPer100ms(double rad_s) {
			return rad_s / (Math.PI * 2.0) * kTicksPerRotation / 10.0;
		}

		public static double ticksToRotations(double ticks) {
			return ticks / kTicksPerRotation;
		}

		public static double ticksToInches(double ticks) {
			return ticksToRotations(ticks) * kWheelCircumference;
		}

		public static int inchesToTicks(double inches) {
			return (int)(inchesToRotations(inches) * kTicksPerRotation);
		}

		public static double ticksPer100msToRotationsPerSecond(double ticks) {
			return ticks / kTicksPerRotation * 10.0;
		}

		public static double ticksPer100msToInchesPerSecond(double ticks) {
			return ticksPer100msToRotationsPerSecond(ticks) * kWheelCircumference;
		}

		public static double ticksPer100msToRadiansPerSecond(double ticks) {
			return ticksPer100msToRotationsPerSecond(ticks) * (Math.PI * 2.0);
		}

	}
}