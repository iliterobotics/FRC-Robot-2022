package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import static com.revrobotics.ControlType.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Conversions;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.ELimelightData.TV;
import static us.ilite.common.types.ELimelightData.TX;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.sensor.EGyro;
import us.ilite.common.types.sensor.EPowerDistPanel;
import static us.ilite.common.types.sensor.EPowerDistPanel.*;
import us.ilite.robot.Robot;
import static us.ilite.robot.Enums.*;
import us.ilite.robot.hardware.*;

/**
 * Class for running all drivetrain train control operations from both autonomous and
 * driver-control.
 * TODO Support for rotation trajectories
 * TODO Turn-to-heading with Motion Magic
 */
public class DriveModule extends Module {
	private final ILog mLogger = Logger.createLog(DriveModule.class);
	// DO NOT CHANGE THE GEAR RATIO
	public static double kGearboxRatio = (10.0 / 40.0) * (14.0 / 40.0);
	// As of 2/9this is for omni wheels on a solid floor
	//public static double kWheelDiameterInches = 6.125;
	public static double kWheelDiameterInches = 5.875;
	public static double kWheelCircumferenceFeet = kWheelDiameterInches*Math.PI/12.0;
	// For position, getPosition() returns raw rotations - so convert that to feet
	public static double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
	public static double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;

	// Actual measured was 5514 with a resting battery voltage of 12.75V
	public static double kDriveTrainMaxVelocityRPM = 5500.0;
	public static Distance kDriveMaxVelocity_measured = Distance.fromFeet(kDriveTrainMaxVelocityRPM*kDriveNEOVelocityFactor);
//	public static Distance kDriveMaxAccel_measured = Distance.fromFeet()
	public static Distance kDriveMaxAccel_simulated = Distance.fromFeet(28.5);

	// This is approx 290 Degrees per second, measured with a Pigeon
	public static double kMaxDegreesPerCycle = Units.radians_to_degrees(5);
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
	public static ProfileGains dPID = new ProfileGains()
			.p(1.0).maxVelocity(kDriveTrainMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
			.maxAccel(56760d)
			.slot(POSITION_PID_SLOT)
			.velocityConversion(kDriveNEOPositionFactor);

	public static ProfileGains vPID = new ProfileGains()
			.f(0.00015)
			.p(0.0001)
			// Enforce a maximum allowed speed, system-wide. DO NOT undo kMaxAllowedVelocityMultiplier without checking with a mentor first.
			.maxVelocity(kDriveTrainMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
			// Divide by the simulated blue nitrile CoF 1.2, multiply by omni (on school floor) theoretical of 0.4
			.maxAccel(kDriveMaxAccel_simulated.feet() / kDriveNEOVelocityFactor / 1.2 * 0.8)
			.slot(VELOCITY_PID_SLOT)
			.velocityConversion(kDriveNEOVelocityFactor);

	public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);
	public static double kTurnSensitivity = 0.85;

	// =============================================================================
	// Heading Gains
	// =============================================================================
	public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
	public static ProfileGains kYawGains = new ProfileGains().f(.15);
	public IMU mGyro;

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
	private PIDController mTargetAngleLockPid;
	private PIDController mYawPid;
//	private PIDController mHoldLeftPositionPid;
//	private PIDController mHoldRightPositionPid;
	private boolean mStartHoldingPosition;

	private double mLeftHoldSetpoint;
	private double mRightHoldSetpoint;
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

	private static final SparkMaxFactory.Configuration kDriveConfig = new SparkMaxFactory.Configuration();
	static {
		kDriveConfig.IDLE_MODE = CANSparkMax.IdleMode.kCoast;
	}

	public DriveModule() {
		mLeftMaster = SparkMaxFactory.createSparkMax(Settings.Hardware.CAN.kDriveLeftMaster, kDriveConfig);
		mLeftFollower = SparkMaxFactory.createSparkMax(Settings.Hardware.CAN.kDriveLeftFollower, kDriveConfig);
		mLeftFollower.follow(mLeftMaster);
		mLeftEncoder = new CANEncoder(mLeftMaster);
		mLeftCtrl = mLeftMaster.getPIDController();
		mLeftCtrl.setOutputRange(-kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM);
		mRightMaster = SparkMaxFactory.createSparkMax(Settings.Hardware.CAN.kDriveRightMaster, kDriveConfig);
		mRightFollower = SparkMaxFactory.createSparkMax(Settings.Hardware.CAN.kDriveRightFollower, kDriveConfig);
		mRightFollower.follow(mRightMaster);
		mRightEncoder = new CANEncoder(mRightMaster);
		mRightCtrl = mRightMaster.getPIDController();
		mRightCtrl.setOutputRange(-kDriveTrainMaxVelocityRPM, kDriveTrainMaxVelocityRPM);
		mRightMaster.setInverted(true);
		mRightFollower.setInverted(true);
		mGyro = new Pigeon(Settings.Hardware.CAN.kPigeon);

//		mGyro = new ADIS16470();

		HardwareUtils.setGains(mLeftCtrl, vPID);
		HardwareUtils.setGains(mRightCtrl, vPID);
		HardwareUtils.setGains(mLeftCtrl, dPID);
		HardwareUtils.setGains(mRightCtrl, dPID);

		//TODO - we want to do use our conversion factor calculated above, but that requires re-turning of F & P
		mLeftEncoder.setPositionConversionFactor(1d);
		mLeftEncoder.setVelocityConversionFactor(1d);
		mRightEncoder.setPositionConversionFactor(1d);
		mRightEncoder.setPositionConversionFactor(1d);
		mLeftMaster.burnFlash();
		mLeftFollower.burnFlash();
		mRightMaster.burnFlash();
		mRightFollower.burnFlash();

	}

	@Override
	public void modeInit(EMatchMode pMode, double pNow) {
		mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput, Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
		mTargetAngleLockPid.setSetpoint(0);
		mTargetAngleLockPid.reset();

		mYawPid = new PIDController(kYawGains,
									-kMaxDegreesPerCycle,
									kMaxDegreesPerCycle,
									Settings.kControlLoopPeriod);
		mYawPid.setOutputRange(-1, 1);
		mYawPid.reset();

		mStartHoldingPosition = false;
		mLeftHoldSetpoint = 0.0;
		mRightHoldSetpoint = 0.0;

		reset();
		HardwareUtils.setGains(mLeftCtrl, vPID);
		HardwareUtils.setGains(mRightCtrl, vPID);
		HardwareUtils.setGains(mLeftCtrl, dPID);
		HardwareUtils.setGains(mRightCtrl, dPID);

		System.err.println(" ==== DRIVE MAX ACCEL (RPM): " + (kDriveMaxAccel_simulated.feet() / kDriveNEOVelocityFactor / 1.2 * 0.4));
	}

	@Override
	public void readInputs(double pNow) {
		mGyro.update(pNow);
		db.drivetrain.set(L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
		db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
		db.drivetrain.set(R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
		db.drivetrain.set(R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
		db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getOutputCurrent());
		db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getOutputCurrent());
		db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getVoltageCompensationNominalVoltage());
		db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getVoltageCompensationNominalVoltage());
		db.drivetrain.set(IS_CURRENT_LIMITING, EPowerDistPanel.isAboveCurrentThreshold(kCurrentLimitAmps, Robot.DATA.pdp, kPdpSlots));
		db.imu.set(EGyro.HEADING_DEGREES, -mGyro.getHeading().getDegrees());
		db.imu.set(EGyro.YAW_OMEGA_DEGREES, mGyro.getYawRate().getDegrees());

	}

	@Override
	public void setOutputs(double pNow) {
		EDriveState mode = db.drivetrain.get(STATE, EDriveState.class);
		// Do this to prevent wonkiness while transitioning autonomous to tele op
		if(mode == null) return;
		SmartDashboard.putNumber("Drive module state", mode.ordinal());
		double turn = db.drivetrain.get(DESIRED_TURN_PCT);
		double throttle = db.drivetrain.get(DESIRED_THROTTLE_PCT);
		switch (mode) {
			case RESET:
				reset();
				break;
			case HOLD:
				if (!mStartHoldingPosition) {
					mLeftHoldSetpoint = mLeftEncoder.getPosition();
					mRightHoldSetpoint = mRightEncoder.getPosition();
					mStartHoldingPosition = true;
				}

				if (mLeftEncoder.getVelocity() < 100) {
					if (Math.abs(mLeftEncoder.getPosition() - mLeftHoldSetpoint) > .15) {
						mLeftCtrl.setReference(mLeftHoldSetpoint, kPosition, POSITION_PID_SLOT, 0);
					}
				} else {
					mLeftCtrl.setReference(0.0, kSmartVelocity, VELOCITY_PID_SLOT, 0);
				}

				if (mRightEncoder.getVelocity() < 100) {
					if (Math.abs(mRightEncoder.getPosition() - mRightHoldSetpoint) > .15) {
						mRightCtrl.setReference(mRightHoldSetpoint, kPosition, POSITION_PID_SLOT, 0);
					}
				} else {
					mRightCtrl.setReference(0.0, kSmartVelocity, VELOCITY_PID_SLOT, 0);
				}
				break;
			case TARGET_ANGLE_LOCK:
				RobotCodex<ELimelightData> targetData = Robot.DATA.limelight;
				double pidOutput = 0;
				if(mTargetAngleLockPid != null && targetData != null && targetData.isSet(TV) && targetData.isSet(TX)) {
					//if there is a target in the limelight's fov, lock onto target using feedback loop
					pidOutput = mTargetAngleLockPid.calculate(-1.0 * targetData.get(TX), pNow - mPreviousTime);
					pidOutput = pidOutput + (Math.signum(pidOutput) * Settings.kTargetAngleLockFrictionFeedforward);
					SmartDashboard.putNumber("Target Angle Lock PID Output", pidOutput);
					db.drivetrain.set(DESIRED_TURN_PCT, pidOutput);
					turn = db.drivetrain.get(DESIRED_TURN_PCT);
				}
				SmartDashboard.putNumber("Target Angle Lock PID Output", pidOutput);
			case VELOCITY:
				mStartHoldingPosition = false;
				mYawPid.setSetpoint(db.drivetrain.get(DESIRED_TURN_PCT) * kMaxDegreesPerCycle);
//				turn = mYawPid.calculate(Robot.DATA.imu.get(EGyro.YAW_DEGREES), pNow);
//				SmartDashboard.putNumber("DESIRED YAW", mYawPid.getSetpoint());
//				SmartDashboard.putNumber("ACTUAL YAW", (Robot.DATA.imu.get(EGyro.YAW_DEGREES)));
				mLeftCtrl.setReference((throttle-turn) * kDriveTrainMaxVelocityRPM, kSmartVelocity, VELOCITY_PID_SLOT, 0);
				mRightCtrl.setReference((throttle+turn) * kDriveTrainMaxVelocityRPM, kSmartVelocity, VELOCITY_PID_SLOT, 0);
				break;
			case PATH_FOLLOWING_BASIC:
			case PATH_FOLLOWING_HELIX:
				mLeftCtrl.setReference(db.drivetrain.get(L_PATH_FT_s) / kDriveNEOVelocityFactor, kVelocity, VELOCITY_PID_SLOT, 0);
				mRightCtrl.setReference(db.drivetrain.get(R_PATH_FT_s) / kDriveNEOVelocityFactor, kVelocity, VELOCITY_PID_SLOT, 0);
				break;
			case PERCENT_OUTPUT:
				mLeftMaster.set(throttle+turn);
				mRightMaster.set(throttle-turn);
				break;
		}
	}

	private void reset() {
		mLeftEncoder.setPosition(0.0);
		mRightEncoder.setPosition(0.0);
	}
}