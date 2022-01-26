package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Enums.EDriveState;
import us.ilite.robot.Robot;

import static us.ilite.common.types.EVisionGoal2020.TV;
import static us.ilite.common.types.EVisionGoal2020.TX;
import static us.ilite.common.types.drive.EDriveData.*;

public class DriveModule extends Module {

	private TalonFX mLeftMaster, mLeftFollower, mRightMaster, mRightFollower;

	private final double kMaxFalconVelocity = 6380;
	private final double kMaxDriveThreshold = 0.5;
	private final double kDriveRampRate = 0.5;

	public static double kGearboxRatio = (10.0 / 40.0) * (14.0 / 40.0);
	// As of 2/9this is for omni wheels on a solid floor
	//public static double kWheelDiameterInches = 6.125;
	public static double kWheelDiameterInches = 5.875;
	public static double kWheelCircumferenceFeet = kWheelDiameterInches*Math.PI/12.0;
	// For position, getPosition() returns raw rotations - so convert that to feet
	public static double kDriveFalconPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
	public static double kDriveFalconVelocityFactor = kDriveFalconPositionFactor / 60.0;

	// Actual measured was 5514 with a resting battery voltage of 12.75V
	public static double kDriveTrainMaxVelocityRPM = 5500.0;
	public static Distance kDriveMaxVelocity_measured = Distance.fromFeet(kDriveTrainMaxVelocityRPM*kDriveFalconVelocityFactor);
	//	public static Distance kDriveMaxAccel_measured = Distance.fromFeet()
	public static Distance kDriveMaxAccel_simulated = Distance.fromFeet(28.5);

	// This is approx 290 Degrees per second, measured with a Pigeon
	// Actual measured was 825 Degrees per second, with a resting battery voltage of 12.57V
	public static double kMaxDegreesPerSecond = 300;
	// This is with the ADIS16470 IMU
	public static Rotation2d kDriveMaxOmega_measured = Rotation2d.fromDegrees(19.1);

	public static double kEffectiveWheelbase = 23.25;
	public static double kCurvatureCircumference = kEffectiveWheelbase * Math.PI;
	public static double kInchesPerCurvatureDegree = kCurvatureCircumference / 360.0;
	public static double kWheelTurnsPerCurvatureDegree = kInchesPerCurvatureDegree / kWheelDiameterInches;

	private final double kPositionConversion = 1;
	private final double kWheelDiameter = 4;
	private final double kWheelCircumference = kWheelDiameter * Math.PI;

	private final int vSlot = 1;
	private final int dSlot = 2;

	public static PIDController vPID, dPID, mTargetAngleLockPid;

	private static double mLeftHoldPosition = 0;
	private static double mRightHoldPosition = 0;
	private static int mCyclesHolding = 0;

	public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);
	public static double kTurnSensitivity = 0.85;

	public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
	public static ProfileGains kYawGains = new ProfileGains().p(0.1);

	private final int kVelocitySlot = 0;
	private final int kPositionSlot = 1;

	private final ProfileGains vGains = new ProfileGains()
			.p(0.0001)
			.i(0.0)
			.d(0.0)
			.maxVelocity(kMaxFalconVelocity*kMaxDriveThreshold)
			.f(0.0)
			.velocityConversion(1)
			.maxAccel(kDriveRampRate)
			.slot(vSlot);
	private final ProfileGains dGains = new ProfileGains()
			.p(0.0001)
			.i(0.0)
			.d(0.0)
			.maxVelocity(kMaxFalconVelocity*kMaxDriveThreshold)
			.f(0.0)
			.positionConversion(1)
			.maxAccel(kDriveRampRate)
			.slot(vSlot);

	public DriveModule() {
		mLeftMaster = new TalonFX(Settings.HW.CAN.kDriveLeftMaster);
		mLeftFollower = new TalonFX(Settings.HW.CAN.kDriveLeftFollower);
		mLeftFollower.follow(mLeftMaster);

		mRightMaster = new TalonFX(Settings.HW.CAN.kDriveRightMaster);
		mRightFollower = new TalonFX(Settings.HW.CAN.kDriveRightFollower);
		mRightFollower.follow(mRightMaster);

		vPID = new PIDController(vGains, -kMaxDriveThreshold*kMaxFalconVelocity, kMaxDriveThreshold*kMaxFalconVelocity, Settings.kControlLoopPeriod);
		dPID = new PIDController(dGains, -kMaxDriveThreshold*kMaxFalconVelocity, kMaxDriveThreshold*kMaxFalconVelocity, Settings.kControlLoopPeriod);
		mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput, Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);

		mLeftMaster.config_kP(kVelocitySlot, 0.0001);
		mLeftMaster.config_kI(kVelocitySlot, 0);
		mLeftMaster.config_kD(kVelocitySlot, 0);

		mRightMaster.config_kP(kVelocitySlot, 0.0001);
		mRightMaster.config_kI(kVelocitySlot, 0);
		mRightMaster.config_kD(kVelocitySlot, 0);
	}

	@Override
	public void modeInit(EMatchMode mode) {
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
		mTargetAngleLockPid.setSetpoint(0);
		mTargetAngleLockPid.reset();
	}

	@Override
	public void readInputs() {
		double rawLeftVelocity = mLeftMaster.getSelectedSensorVelocity();
		double rpsLeft = (rawLeftVelocity / 2048) / 1000;
		double feetPerSecondLeft = rpsLeft / kWheelCircumference;

		double rawRightVelocity = mRightMaster.getSelectedSensorVelocity();
		double rpsRight = (rawRightVelocity / 2048) / 1000;
		double feetPerSecondRight = rpsRight / kWheelCircumference;

		double rawPositionLeft = mLeftMaster.getSelectedSensorPosition();
		double rotLeft = (rawPositionLeft) / 2048;
		double feetLeft = rotLeft / kWheelCircumference;

		double rawPositionRight = mRightMaster.getSelectedSensorPosition();
		double rotRight = (rawPositionRight) / 2048;
		double feetRight = rotRight / kWheelCircumference;

		db.drivetrain.set(L_ACTUAL_POS_FT, feetLeft);
		db.drivetrain.set(R_ACTUAL_POS_FT, feetRight);
		db.drivetrain.set(L_ACTUAL_VEL_FT_s, feetPerSecondLeft);
		db.drivetrain.set(R_ACTUAL_VEL_FT_s, feetPerSecondRight);
	}

	@Override
	public void setOutputs() {
		EDriveState state = db.drivetrain.get(STATE, EDriveState.class);
		double throttle = db.drivetrain.get(DESIRED_THROTTLE_PCT);
		double turn = db.drivetrain.get(DESIRED_TURN_PCT);

		double left = throttle + turn;
		double right = throttle - turn;

		if(state == null) return;

		switch(state) {
			case RESET:
				reset();
				break;
			case PERCENT_OUTPUT:
				mLeftMaster.set(ControlMode.PercentOutput, left);
				mRightMaster.set(ControlMode.PercentOutput, right);
				break;
			case VELOCITY:
				double leftPIDValue =  vPID.calculate(left*kDriveTrainMaxVelocityRPM, clock.dt());
				double rightPIDValue =  vPID.calculate(right*kDriveTrainMaxVelocityRPM, clock.dt());

				mLeftMaster.set(ControlMode.Velocity,left*kDriveTrainMaxVelocityRPM);
				mRightMaster.set(ControlMode.Velocity, right*kDriveTrainMaxVelocityRPM);
				mCyclesHolding = 0;
				break;
			case TARGET_ANGLE_LOCK:
				double pidOutput = 0;
				if(mTargetAngleLockPid != null && db.goaltracking != null && db.goaltracking.isSet(TV) && db.goaltracking.isSet(TX)) {
					//if there is a target in the limelight's fov, lock onto target using feedback loop
					pidOutput = mTargetAngleLockPid.calculate(-1.0 * db.goaltracking.get(TX), clock.dt());
					pidOutput = pidOutput + (Math.signum(pidOutput) * Settings.kTargetAngleLockFrictionFeedforward);
//					SmartDashboard.putNumber("Target Angle Lock PID Output", pidOutput);
					turn = pidOutput;
				}
				mLeftMaster.set(ControlMode.Velocity, pidOutput);
				mRightMaster.set(ControlMode.Velocity, -pidOutput);
				break;
			case HOLD:
				if (mCyclesHolding == 0) {
					mLeftHoldPosition = db.drivetrain.get(L_ACTUAL_POS_FT);
					mRightHoldPosition = db.drivetrain.get(R_ACTUAL_POS_FT);
				}

				double mCurrentLeftPosition = db.drivetrain.get(L_ACTUAL_POS_FT);
				double mCurrentRightPosition = db.drivetrain.get(R_ACTUAL_POS_FT);

				double deltaLeft = mCurrentLeftPosition - mLeftHoldPosition;
				double deltaRight = mCurrentRightPosition - mRightHoldPosition;

				if (db.drivetrain.get(L_ACTUAL_VEL_FT_s) < 100) {
					if (Math.abs(deltaLeft) >= 0.1) {
						mLeftMaster.set(ControlMode.Position, dPID.calculate(mLeftHoldPosition, clock.dt()));
					} else {
						mLeftMaster.set(ControlMode.Velocity, 0);
					}
				}

				if (db.drivetrain.get(R_ACTUAL_VEL_FT_s) < 100) {
					if (Math.abs(deltaRight) >= 0.1) {
						mRightMaster.set(ControlMode.Position, dPID.calculate(mRightHoldPosition, clock.dt()));
					} else {
						mRightMaster.set(ControlMode.Velocity, 0);
					}
				}

				mCyclesHolding++;
				break;
			case SMART_MOTION:
				mLeftMaster.set(ControlMode.Position, dPID.calculate(db.drivetrain.get(L_DESIRED_POS), clock.dt()));
				mRightMaster.set(ControlMode.Position, dPID.calculate(db.drivetrain.get(R_DESIRED_POS), clock.dt()));
				break;
			default:
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
		}
	}

	private void reset() {

	}
}
