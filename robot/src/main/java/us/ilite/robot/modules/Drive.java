package us.ilite.robot.modules;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import us.ilite.common.Data;
import us.ilite.common.config.AbstractSystemSettingsUtils;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.hardware.*;
import us.ilite.robot.loops.Loop;

/**
 * Class for running all drive train control operations from both autonomous and
 * driver-control.
 * TODO Support for rotation trajectories
 * TODO Turn-to-heading with Motion Magic
 */
public class Drive extends Loop {
	private final ILog mLogger = Logger.createLog(Drive.class);

	private Data mData;

	private IDriveHardware mDriveHardware;
	private Rotation2d mGyroOffset = new Rotation2d();

	private EDriveState mDriveState;
	private DriveMessage mDriveMessage;
	private double mTargetTrackingThrottle = 0;

	private PIDController mTargetAngleLockPid;

	private double mPreviousTime = 0;

	public Drive(Data data)
	{
		this.mData = data;
		if(AbstractSystemSettingsUtils.isPracticeBot()) {
		} else {
			this.mDriveHardware = new NeoDriveHardware(Settings.Drive.kGearboxRatio);
		}

		this.mDriveHardware.init();
	}

	@Override
	public void modeInit(double pNow) {
		mTargetAngleLockPid = new PIDController(Settings.kTargetAngleLockGains, Settings.kTargetAngleLockMinInput, Settings.kTargetAngleLockMaxInput, Settings.kControlLoopPeriod);
		mTargetAngleLockPid.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);
		mTargetAngleLockPid.setSetpoint(0);
		mTargetAngleLockPid.reset();
		mDriveHardware.zero();

	  	setDriveMessage(DriveMessage.kNeutral);
	  	setDriveState(EDriveState.NORMAL);

//	  	startCsvLogging();
	}

	@Override
	public void periodicInput(double pNow) {

		mData.drive.set(EDriveData.LEFT_POS_INCHES, mDriveHardware.getLeftInches());
		mData.drive.set(EDriveData.RIGHT_POS_INCHES, mDriveHardware.getRightInches());
//		mData.drive.set(EDriveData.LEFT_VEL_IPS, mDriveHardware.getLeftVelInches());
//		mData.drive.set(EDriveData.RIGHT_VEL_IPS, mDriveHardware.getRightVelInches());
		mData.drive.set(EDriveData.LEFT_VEL_TICKS, (double)mDriveHardware.getLeftVelTicks());
		mData.drive.set(EDriveData.RIGHT_VEL_TICKS, (double)mDriveHardware.getRightVelTicks());

		mData.drive.set(EDriveData.LEFT_MESSAGE_OUTPUT, mDriveMessage.getLeftOutput());
		mData.drive.set(EDriveData.RIGHT_MESSAGE_OUTPUT, mDriveMessage.getRightOutput());
		mData.drive.set(EDriveData.LEFT_MESSAGE_CONTROL_MODE, (double)mDriveMessage.getMode().ordinal());
		mData.drive.set(EDriveData.RIGHT_MESSAGE_CONTROL_MODE, (double)mDriveMessage.getMode().ordinal());
		mData.drive.set(EDriveData.LEFT_MESSAGE_NEUTRAL_MODE, (double)mDriveMessage.getNeutral().ordinal());
		mData.drive.set(EDriveData.RIGHT_MESSAGE_NEUTRAL_MODE, (double)mDriveMessage.getNeutral().ordinal());
//
		mData.imu.set(EGyro.YAW_DEGREES, mDriveHardware.getImu().getHeading().getDegrees());

//		SimpleNetworkTable.writeCodexToSmartDashboard(EDriveData.class, mData.drive, mClock.getCurrentTime());
	}

	@Override
	public void update(double pNow) {
        if(mDriveState != EDriveState.NORMAL) {
			mLogger.error("Invalid drive state - maybe you meant to run this a high frequency?");
			mDriveState = EDriveState.NORMAL;
		} else {
			mDriveHardware.set(mDriveMessage);
		}

		mPreviousTime = pNow;
	}
	
	@Override
	public void shutdown(double pNow) {
		mDriveHardware.zero();
	}

	@Override
	public void loop(double pNow) {
//		mUpdateTimer.start();
		switch(mDriveState) {
			case PATH_FOLLOWING:
			case TARGET_ANGLE_LOCK:

				Codex<Double, ETargetingData> targetData = mData.limelight;
				double pidOutput;
				if(mTargetAngleLockPid != null && targetData != null && targetData.isSet(ETargetingData.tv) && targetData.get(ETargetingData.tx) != null) {

					//if there is a target in the limelight's fov, lock onto target using feedback loop
					pidOutput = mTargetAngleLockPid.calculate(-1.0 * targetData.get(ETargetingData.tx), pNow - mPreviousTime);
					pidOutput = pidOutput + (Math.signum(pidOutput) * Settings.kTargetAngleLockFrictionFeedforward);

					mDriveMessage = new DriveMessage().throttle(mTargetTrackingThrottle).turn(pidOutput).calculateCurvature();
					// If we've already seen the target and lose tracking, exit.
				}

				break;
			case NORMAL:
				break;
			default:
				mLogger.warn("Got drive state: " + mDriveState+" which is unhandled");
				break;
		}
		mDriveHardware.set(mDriveMessage);
		mPreviousTime = pNow;
//		mUpdateTimer.stop();
	}

	public synchronized void setTargetAngleLock() {
		mDriveState = EDriveState.TARGET_ANGLE_LOCK;
		mDriveHardware.configureMode(ECommonControlMode.PERCENT_OUTPUT);
		mDriveHardware.set(DriveMessage.kNeutral);
	}

	public synchronized void setPathFollowing() {
		mDriveState = EDriveState.PATH_FOLLOWING;
		mDriveHardware.configureMode(ECommonControlMode.VELOCITY);
	}

	public synchronized void setNormal() {
		mDriveState = EDriveState.NORMAL;
	}

	public synchronized void setTargetTrackingThrottle(double pTargetTrackingThrottle) {
		mTargetTrackingThrottle = pTargetTrackingThrottle;
	}


	@Override
	public boolean checkModule(double pNow) {
        return mDriveHardware.checkHardware();
	}


	public synchronized void zero() {
		mDriveHardware.zero();
	}

	private void setDriveState(EDriveState pDriveState) {
		this.mDriveState = pDriveState;
	}

	public synchronized void setDriveMessage(DriveMessage pDriveMessage) {
		this.mDriveMessage = pDriveMessage;
	}


	public synchronized IDriveHardware getDriveHardware() {
	    return mDriveHardware;
    }

    public synchronized DriveMessage getDriveMessage() {
		return mDriveMessage;
	}

	public boolean isCurrentLimiting() {
		return EPowerDistPanel.isAboveCurrentThreshold(Settings.Drive.kCurrentLimitAmps, mData.pdp, Settings.Drive.kPdpSlots);
	}

}