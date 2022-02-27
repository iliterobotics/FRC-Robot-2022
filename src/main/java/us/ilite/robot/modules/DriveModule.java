package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.hardware.Pigeon;

import java.util.Set;

import static us.ilite.common.types.ELimelightData.*;
import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.sensor.EPowerDistPanel.*;
import static us.ilite.common.types.sensor.EPowerDistPanel.CURRENT14;

public class DriveModule extends Module {

	private final TalonFX mLeftMaster;
	private final TalonFX mLeftFollower;
	private final TalonFX mRightMaster;
	private final TalonFX mRightFollower;
	private final Encoder mLeftEncoder;
	private final Encoder mRightEncoder;
	private static Pigeon mGyro;

	// ========================================
	// DO NOT MODIFY THESE CONSTANTS
	// ========================================
	public static final double kGearboxRatio = 12.0 / 40.0 * 14.0 / 40.0;
	public static final double kWheelCircumferenceFeet = 3.9 / 12.0 * Math.PI;
	public static final double kUnitsToScaledRPM = 600.0 / 2048.0 * kGearboxRatio;
	public static final double kRPMtoFTs = (kUnitsToScaledRPM * kWheelCircumferenceFeet) / 60.0;
	public static final double kTrackWidthFeet = 22.0 / 12.0;
	public static final double kWheelBaseDiagonalFeet = 35.0 / 12.0;
	public static final double kMaxDriveThreshold = 0.75;
	public static final double kMaxDriveVelocity = 6380.0;
	public static final double kMaxDriveVelocityFTs = kMaxDriveVelocity * kRPMtoFTs;
	public static final double kDriveRampRate = kMaxDriveVelocity / 5;
	public static final double kUnitsToScaledRotationsPosition = 2048.0 / kGearboxRatio;
	public static final double kPulsesPerRotation = 256;
	public static final int kCurrentLimitAmps = 50;

	public static EPowerDistPanel[] kPdpSlots = new EPowerDistPanel[]{
			/* Left */
			CURRENT1,
			CURRENT2,

			/* Right */
			CURRENT13,
			CURRENT14,
	};

	private final int vSlot = 1;
	private final int dSlot = 2;

	public static PIDController mLeftVelocityPID;
	public static PIDController mRightVelocityPID;
	public static PIDController mRightPositionPID;
	public static PIDController mLeftPositionPID;
	public static PIDController mTargetAngleLockPid;

	private static DifferentialDriveOdometry mOdometry;
	private static DifferentialDrive mDifferentialDrive;

	private static double mLeftHoldPosition = 0;
	private static double mRightHoldPosition = 0;
	private static int mCyclesHolding = 0;
	private static double mLastHeading = 0;
	private static double mDeltaTime = 0;

	public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);
	public static double kTurnSensitivity = 0.85;

	public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
	public static ProfileGains kTargetAngleLockGains = new ProfileGains().p(0.1);

	public static double kInitialLeftPosition = 0;
	public static double kInitialRightPosition = 0;
	public static double kInitialXPosition = 0;
	public static double kInitialYPosition = 0;

	private final ProfileGains kVelocityGains = new ProfileGains()
			.p(0.0005)
			.i(0.0)
			.d(0.0)
			.maxVelocity(kMaxDriveVelocityFTs)
			.f(0.5)
			.velocityConversion(1)
			.maxAccel(kDriveRampRate)
			.slot(vSlot);
	private final ProfileGains kPositionGains = new ProfileGains()
			.p(0.1)
			.maxVelocity(kMaxDriveVelocityFTs)
			.positionConversion(1)
			.maxAccel(kDriveRampRate)
			.slot(dSlot);

	public DriveModule() {
		mLeftMaster = new TalonFX(Settings.HW.CAN.kDTML1);
		mLeftFollower = new TalonFX(Settings.HW.CAN.kDTL3);
		mLeftFollower.follow(mLeftMaster);

		mLeftMaster.setNeutralMode(NeutralMode.Coast);
		mLeftFollower.setNeutralMode(NeutralMode.Coast);

		mRightMaster = new TalonFX(Settings.HW.CAN.kDTMR2);
		mRightFollower = new TalonFX(Settings.HW.CAN.kDTR4);
		mRightFollower.follow(mRightMaster);

		mRightMaster.setNeutralMode(NeutralMode.Coast);
		mRightFollower.setNeutralMode(NeutralMode.Coast);

		mRightMaster.setInverted(true);
		mRightFollower.setInverted(true);

		mGyro = new Pigeon(Robot.CLOCK, Settings.HW.CAN.kDTGyro);

		mLeftEncoder = new Encoder(Settings.HW.DIO.kEDTLA, Settings.HW.DIO.kEDTLB);
		mRightEncoder = new Encoder(Settings.HW.DIO.kEDTRA, Settings.HW.DIO.kEDTRB);

		mLeftVelocityPID = new PIDController(kVelocityGains, -kMaxDriveVelocityFTs, kMaxDriveVelocityFTs, Settings.kControlLoopPeriod);
		mLeftVelocityPID.setOutputRange(-1,
				1);
		mRightVelocityPID = new PIDController(kVelocityGains, -kMaxDriveVelocityFTs, kMaxDriveVelocityFTs, Settings.kControlLoopPeriod);
		mRightVelocityPID.setOutputRange(-1,
				1);

		mLeftPositionPID = new PIDController(kPositionGains, -kMaxDriveVelocityFTs, kMaxDriveVelocityFTs, Settings.kControlLoopPeriod);
		mLeftPositionPID.setOutputRange(-1 * Settings.Input.kMaxAllowedVelocityMultiplier,
				1 * Settings.Input.kMaxAllowedVelocityMultiplier);
		mRightPositionPID = new PIDController(kPositionGains, -kMaxDriveVelocityFTs, kMaxDriveVelocityFTs, Settings.kControlLoopPeriod);
		mRightPositionPID.setOutputRange(-1 * Settings.Input.kMaxAllowedVelocityMultiplier,
				1 * Settings.Input.kMaxAllowedVelocityMultiplier);

		mTargetAngleLockPid = new PIDController(kTargetAngleLockGains, -kMaxDriveVelocityFTs, kMaxDriveVelocityFTs, Settings.kControlLoopPeriod);
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);

		//TODO figure out a way to call mDrive.feed() using TalonFX
//		mDifferentialDrive = new DifferentialDrive((MotorController) mLeftMaster, (MotorController) mRightMaster);
		mOdometry = new DifferentialDriveOdometry(mGyro.getHeading());
	}

	@Override
	public void modeInit(EMatchMode mode) {
		reset();
		resetOdometry(TrajectoryCommandUtils.getJSONTrajectory().getInitialPose());
		kInitialXPosition = mOdometry.getPoseMeters().getX();
		kInitialYPosition = mOdometry.getPoseMeters().getY();
		kInitialLeftPosition = mLeftMaster.getSelectedSensorPosition();
		kInitialRightPosition = mRightMaster.getSelectedSensorPosition();
	}

	@Override
	public void readInputs() {
		mGyro.update();
		db.drivetrain.set(DELTA_HEADING, mGyro.getHeading().getDegrees() - mLastHeading);
		db.drivetrain.set(GYRO_RATE, db.drivetrain.get(DELTA_HEADING) / mDeltaTime);
		db.drivetrain.set(ACTUAL_HEADING_RADIANS, mGyro.getHeading().getRadians());
		db.drivetrain.set(ACTUAL_HEADING_DEGREES, mGyro.getHeading().getRadians());
		db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getBusVoltage());
		db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getBusVoltage());
		db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getStatorCurrent());
		db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getStatorCurrent());
		db.drivetrain.set(L_ACTUAL_VEL_RPM, mLeftMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM);
		db.drivetrain.set(R_ACTUAL_VEL_RPM, mRightMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM);
		db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs);
		db.drivetrain.set(R_ACTUAL_VEL_FT_s, mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs);
		db.drivetrain.set(L_ACTUAL_POS_FT, ((mLeftMaster.getSelectedSensorPosition() - kInitialLeftPosition) / kUnitsToScaledRotationsPosition) * kWheelCircumferenceFeet);
		db.drivetrain.set(R_ACTUAL_POS_FT, ((mRightMaster.getSelectedSensorPosition() - kInitialRightPosition) / kUnitsToScaledRotationsPosition) * kWheelCircumferenceFeet);
		db.drivetrain.set(L_ACTUAL_VEL_meters_s, Units.feet_to_meters(mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs));
		db.drivetrain.set(R_ACTUAL_VEL_meters_s, Units.feet_to_meters(mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs));
		db.drivetrain.set(L_ACTUAL_POS_meters, Units.feet_to_meters(((mLeftMaster.getSelectedSensorPosition() - kInitialLeftPosition) / kUnitsToScaledRotationsPosition) * kWheelCircumferenceFeet));
		db.drivetrain.set(R_ACTUAL_POS_meters, Units.feet_to_meters(((mRightMaster.getSelectedSensorPosition() - kInitialRightPosition) / kUnitsToScaledRotationsPosition) * kWheelCircumferenceFeet));
		db.drivetrain.set(ACTUAL_LEFT_PCT, (mLeftMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM) / (kMaxDriveVelocity * kGearboxRatio));
		db.drivetrain.set(ACTUAL_RIGHT_PCT, (mRightMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM) / (kMaxDriveVelocity * kGearboxRatio));
		db.drivetrain.set(IS_CURRENT_LIMITING, EPowerDistPanel.isAboveCurrentThreshold(kCurrentLimitAmps, Robot.DATA.pdp, kPdpSlots));


		double odoX = mOdometry.getPoseMeters().getX() - kInitialXPosition;
		double odoY = mOdometry.getPoseMeters().getY() - kInitialYPosition;
		db.drivetrain.set(GET_X_OFFSET_METERS, odoX);
		db.drivetrain.set(GET_Y_OFFSET_METERS, odoY);

		db.drivetrain.set(GREYHILL_ACTUAL_LEFT_ft, (mLeftEncoder.getDistance() / kPulsesPerRotation) * kWheelCircumferenceFeet);
		db.drivetrain.set(GREYHILL_ACTUAL_RIGHT_ft, (mRightEncoder.getDistance() / kPulsesPerRotation) * kWheelCircumferenceFeet);
		db.drivetrain.set(GREYHILL_ACTUAL_LEFT_meters, Units.feet_to_meters(mLeftEncoder.getDistance() / kPulsesPerRotation) * kWheelCircumferenceFeet);
		db.drivetrain.set(GREYHILL_ACTUAL_RIGHT_meters, Units.feet_to_meters(mRightEncoder.getDistance() / kPulsesPerRotation) * kWheelCircumferenceFeet);
		//TODO change to greyhill once we get that working
		mOdometry.update(mGyro.getHeading(), Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_POS_FT)),
				Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_POS_FT)));
		Robot.FIELD.setRobotPose(mOdometry.getPoseMeters());
	}

	@Override
	public void setOutputs() {
		Enums.EDriveState state = db.drivetrain.get(STATE, Enums.EDriveState.class);
		double throttle = db.drivetrain.get(DESIRED_THROTTLE_PCT);
		double turn = db.drivetrain.get(DESIRED_TURN_PCT);

		double left = throttle + turn;
		double right = throttle - turn;

		if (state == null) return;

		switch(state) {
			case RESET:
				reset();
				break;
			case PERCENT_OUTPUT:
				mLeftMaster.set(ControlMode.PercentOutput, left);
				mRightMaster.set(ControlMode.PercentOutput, right);
				break;
			case VELOCITY:
				double actualLeft = db.drivetrain.get(L_ACTUAL_VEL_FT_s);
				double actualRight = db.drivetrain.get(R_ACTUAL_VEL_FT_s);

				mLeftVelocityPID.setSetpoint(left * kMaxDriveVelocityFTs * Settings.Input.kMaxAllowedVelocityMultiplier);
				mRightVelocityPID.setSetpoint(right * kMaxDriveVelocityFTs * Settings.Input.kMaxAllowedVelocityMultiplier);

				double leftPIDValue =  mLeftVelocityPID.calculate(actualLeft, clock.getCurrentTimeInMillis());
				double rightPIDValue =  mRightVelocityPID.calculate(actualRight, clock.getCurrentTimeInMillis());

				mLeftMaster.set(ControlMode.PercentOutput, leftPIDValue);
				mRightMaster.set(ControlMode.PercentOutput, rightPIDValue);

				mCyclesHolding = 0;
				break;
			case HOLD:
				if (mCyclesHolding == 0) {
					mLeftHoldPosition = db.drivetrain.get(L_ACTUAL_POS_FT);
					mRightHoldPosition = db.drivetrain.get(R_ACTUAL_POS_FT);
				}

				double mCurrentLeftPosition = db.drivetrain.get(L_ACTUAL_POS_FT);
				double mCurrentRightPosition = db.drivetrain.get(R_ACTUAL_POS_FT);

				mLeftVelocityPID.setSetpoint(mLeftHoldPosition);
				mRightVelocityPID.setSetpoint(mRightHoldPosition);

				double deltaLeft = mCurrentLeftPosition - mLeftHoldPosition;
				double deltaRight = mCurrentRightPosition - mRightHoldPosition;

				if (db.drivetrain.get(L_ACTUAL_VEL_FT_s) < 2) {
					if (Math.abs(deltaLeft) >= 0.1) {
						mLeftMaster.set(ControlMode.Position, mLeftPositionPID.calculate(mLeftHoldPosition, clock.dt()));
					} else {
						mLeftMaster.set(ControlMode.Velocity, 0);
					}
				}
				if (db.drivetrain.get(R_ACTUAL_VEL_FT_s) < 2) {
					if (Math.abs(deltaRight) >= 0.1) {
						mRightMaster.set(ControlMode.Position, mRightPositionPID.calculate(mRightHoldPosition, clock.dt()));

					} else {
						mRightMaster.set(ControlMode.Velocity, 0);
					}
				}
				mCyclesHolding++;
				break;
			case POSITION:
				mLeftPositionPID.setSetpoint(db.drivetrain.get(L_DESIRED_POS_FT));
				mRightPositionPID.setSetpoint(db.drivetrain.get(R_DESIRED_POS_FT));
				double desiredLeft = mLeftPositionPID.calculate(db.drivetrain.get(L_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
				double desiredRight = mRightPositionPID.calculate(db.drivetrain.get(R_ACTUAL_POS_FT), clock.getCurrentTimeInMillis());
				mLeftMaster.set(ControlMode.PercentOutput, desiredLeft);
				mRightMaster.set(ControlMode.PercentOutput, desiredRight);
				break;
			case PATH_FOLLOWING_RAMSETE:
				mLeftMaster.set(ControlMode.PercentOutput, db.drivetrain.get(DESIRED_LEFT_ft_s) / kMaxDriveVelocityFTs);
				mRightMaster.set(ControlMode.PercentOutput, db.drivetrain.get(DESIRED_RIGHT_ft_s) / kMaxDriveVelocityFTs);
//				mDifferentialDrive.feed();
				break;
			default:
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
		}
	}
	public static void resetOdometry(Pose2d pose) {
		mOdometry.resetPosition(pose, mGyro.getHeading());
	}

	private void reset() {
		mLeftMaster.set(ControlMode.Position, 0.0);
		mRightMaster.set(ControlMode.Position, 0.0);
	}
}
