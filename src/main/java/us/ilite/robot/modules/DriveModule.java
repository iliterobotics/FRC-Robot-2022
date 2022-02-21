package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import static us.ilite.common.types.ELimelightData.*;
import static us.ilite.common.types.drive.EDriveData.*;

public class DriveModule extends Module {

	private TalonFX mLeftMaster, mLeftFollower, mRightMaster, mRightFollower;
	private Encoder mLeftEncoder, mRightEncoder;

	public static final double kGearboxRatio = 12.0 / 40.0 * 14.0 / 40.0;
	public static final double kWheelCircumferenceFeet = 3.9 / 12.0 * Math.PI;
	public static final double kUnitsToScaledRPM = 600.0 / 2048.0 * kGearboxRatio;
	public static final double kRPMtoFTs = (kUnitsToScaledRPM * kWheelCircumferenceFeet) / 60.0;
	public static final double kTrackWidthFeet = 22.0 / 12.0;
	public static final double kWheelBaseDiagonalFeet = 35.0 / 12.0;
	public static final double kMaxDriveThreshold = 0.75;
	public static final double kMaxDriveVelocity = 6380.0;
	public static final double kDriveRampRate = kMaxDriveVelocity / 5;

	private final int vSlot = 1;
	private final int dSlot = 2;

	public static PIDController velocityPID, positionPID, mTargetAngleLockPid;

	private static double mLeftHoldPosition = 0;
	private static double mRightHoldPosition = 0;
	private static int mCyclesHolding = 0;

	public static ProfileGains kTurnToProfileGains = new ProfileGains().f(0.085);
	public static double kTurnSensitivity = 0.85;

	public static ProfileGains kDriveHeadingGains = new ProfileGains().p(0.03);
	public static ProfileGains kYawGains = new ProfileGains().p(0.1);

	private final ProfileGains kVelocityGains = new ProfileGains()
			.p(0.0001)
			.i(0.0)
			.d(0.0)
			.maxVelocity(kMaxDriveVelocity * kMaxDriveThreshold * kGearboxRatio)
			.f(0.0)
			.velocityConversion(1)
			.maxAccel(kDriveRampRate)
			.slot(vSlot);
	private final ProfileGains kPositionGains = new ProfileGains()
			.p(0.0001)
			.i(0.0)
			.d(0.0)
			.maxVelocity(kMaxDriveVelocity * kMaxDriveThreshold * kGearboxRatio)
			.f(0.0)
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

		mLeftEncoder = new Encoder(Settings.HW.DIO.kEDTLA, Settings.HW.DIO.kEDTLB);
		mRightEncoder = new Encoder(Settings.HW.DIO.kEDTRA, Settings.HW.DIO.kEDTRB);

		velocityPID = new PIDController(kVelocityGains, -kMaxDriveThreshold * kMaxDriveVelocity * kGearboxRatio,
				kMaxDriveThreshold * kMaxDriveVelocity * kGearboxRatio, Settings.kControlLoopPeriod);
		positionPID = new PIDController(kPositionGains, -kMaxDriveThreshold * kMaxDriveVelocity * kGearboxRatio,
				kMaxDriveThreshold * kMaxDriveVelocity * kGearboxRatio, Settings.kControlLoopPeriod);
		mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput,
				Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);
	}

	@Override
	public void modeInit(EMatchMode mode) {
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
		mTargetAngleLockPid.setSetpoint(0);
		mTargetAngleLockPid.reset();
		reset();
	}

	@Override
	public void readInputs() {
		db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getBusVoltage());
		db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getBusVoltage());
		db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getStatorCurrent());
		db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getStatorCurrent());
		db.drivetrain.set(L_ACTUAL_VEL_RPM, mLeftMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM);
		db.drivetrain.set(R_ACTUAL_VEL_RPM, mRightMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM);
		db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs);
		db.drivetrain.set(R_ACTUAL_VEL_FT_s, mLeftMaster.getSelectedSensorVelocity() * kRPMtoFTs);
		db.drivetrain.set(L_ACTUAL_POS_FT, mLeftMaster.getSelectedSensorPosition() * kRPMtoFTs * 60);
		db.drivetrain.set(R_ACTUAL_POS_FT, mRightMaster.getSelectedSensorPosition() * kRPMtoFTs * 60);
		db.drivetrain.set(ACTUAL_LEFT_PCT, (mLeftMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM) / (kMaxDriveVelocity * kGearboxRatio));
		db.drivetrain.set(ACTUAL_RIGHT_PCT, (mRightMaster.getSelectedSensorVelocity() * kUnitsToScaledRPM) / (kMaxDriveVelocity * kGearboxRatio));
	}

	@Override
	public void setOutputs() {
		//TODO fix velocity mode and position mode
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
				double leftPIDValue =  velocityPID.calculate(left * kMaxDriveVelocity * kGearboxRatio, clock.dt());
				double rightPIDValue =  velocityPID.calculate(right * kMaxDriveVelocity * kGearboxRatio, clock.dt());
				mLeftMaster.set(ControlMode.Velocity, left * kMaxDriveVelocity * kGearboxRatio);
				mRightMaster.set(ControlMode.Velocity, right * kMaxDriveVelocity * kGearboxRatio);
				mCyclesHolding = 0;
				break;
			case TARGET_ANGLE_LOCK:
				double pidOutput = 0;
				if(mTargetAngleLockPid != null && db.limelight != null && db.limelight.isSet(TV) && db.limelight.isSet(TX)) {
					//if there is a target in the limelight's fov, lock onto target using feedback loop
					pidOutput = mTargetAngleLockPid.calculate(-1.0 * db.limelight.get(TX), clock.dt());
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
						mLeftMaster.set(ControlMode.Position, positionPID.calculate(mLeftHoldPosition, clock.dt()));
					} else {
						mLeftMaster.set(ControlMode.Velocity, 0);
					}
				}

				if (db.drivetrain.get(R_ACTUAL_VEL_FT_s) < 100) {
					if (Math.abs(deltaRight) >= 0.1) {
						mRightMaster.set(ControlMode.Position, positionPID.calculate(mRightHoldPosition, clock.dt()));
					} else {
						mRightMaster.set(ControlMode.Velocity, 0);
					}
				}

				mCyclesHolding++;
				break;
			case POSITION:
				mLeftMaster.set(ControlMode.Position, positionPID.calculate(db.drivetrain.get(L_DESIRED_POS), clock.dt()));
				mRightMaster.set(ControlMode.Position, positionPID.calculate(db.drivetrain.get(R_DESIRED_POS), clock.dt()));
				break;
			default:
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
				mLeftMaster.set(ControlMode.PercentOutput, 0.0);
		}
	}
	private void reset() {
		mLeftMaster.set(ControlMode.Position, 0.0);
		mRightMaster.set(ControlMode.Position, 0.0);
	}
}
