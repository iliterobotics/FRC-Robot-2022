package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ETrackingType;
import us.ilite.common.types.input.EInputScale;
import us.ilite.common.types.input.ELogitech310;
import static us.ilite.robot.hardware.ECommonControlMode.*;

import us.ilite.robot.commands.LimelightTargetLock;
import us.ilite.robot.modules.Module;
import us.ilite.robot.modules.*;

public class DriverInput extends Module implements IThrottleProvider, ITurnProvider {

    protected static final double
            DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(DriverInput.class);


    protected DriveModule mDrive;
    private CommandManager mTeleopCommandManager;
    private CommandManager mAutonomousCommandManager;
    private Limelight mLimelight;
    private Data mData;
    private Timer mGroundCargoTimer = new Timer();

    private boolean mIsCargo = false;
    private Joystick mDriverJoystick;
    private Joystick mOperatorJoystick;

    protected Codex<Double, ELogitech310> mDriverInputCodex, mOperatorInputCodex;

    private ETrackingType mTrackingType;
    private ETrackingType mLastTrackingType = null;

    private double mLimelightZoomThreshold = 7.0;


    public DriverInput(DriveModule pDrivetrain, Limelight pLimelight, Data pData,
                       CommandManager pTeleopCommandManager, CommandManager pAutonomousCommandManager,
                       boolean pSimulated) {
        this.mLimelight = pLimelight;
        this.mData = pData;
        this.mTeleopCommandManager = pTeleopCommandManager;
        this.mAutonomousCommandManager = pAutonomousCommandManager;

        this.mDriverInputCodex = mData.driverinput;
        this.mOperatorInputCodex = mData.operatorinput;
        if(pSimulated) {
            // Use a different joystick library?

        } else {
            this.mDriverJoystick = new Joystick(0);
            this.mOperatorJoystick = new Joystick(1);
        }
    }


    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        ELogitech310.map(mData.driverinput, mDriverJoystick);
        ELogitech310.map(mData.operatorinput, mOperatorJoystick);
    }

    @Override
    public void setOutputs(double pNow) {
        /*
        If the driver started the commands that the superstructure is running and then released the button,
        stop running commands.
        */


    }

    private void updateDriveTrain() {
        double rotate = getTurn();
        double throttle = getThrottle();

        //		    throttle = EInputScale.EXPONENTIAL.map(throttle, 2);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate *= Settings.Input.kNormalPercentThrottleReduction;

        if (mData.driverinput.isSet(InputMap.DRIVER.SUB_WARP_AXIS) && mData.driverinput.get(InputMap.DRIVER.SUB_WARP_AXIS) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
            throttle *= Settings.Input.kSnailModePercentThrottleReduction;
            rotate *= Settings.Input.kSnailModePercentRotateReduction;
        }

        DriveMessage driveMessage = new DriveMessage().throttle(throttle).turn(rotate).mode(PERCENT_OUTPUT).normalize().calculateCurvature();

        mDrive.setDriveMessage(driveMessage);
    }
    private void updateLimelightTargetLock(double pNow) {
        if ( mDriverInputCodex.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)){
            if (mData.selectedTarget.get(ELimelightData.ty) != null) {
                SmartDashboard.putNumber("Distance to Target", mLimelight.calcTargetDistance(72));
            }
            mTrackingType = ETrackingType.TARGET;
        }
        else if (mDriverInputCodex.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)){
            if (mData.selectedTarget.get(ELimelightData.ty) != null) {
                if (Math.abs(mData.selectedTarget.get(ELimelightData.tx)) < mLimelightZoomThreshold) {
                    mLimelight.setTracking(ETrackingType.TARGET_ZOOM);
                    System.out.println("ZOOMING");
                } else {
                    mLimelight.setTracking(ETrackingType.TARGET);
                }
            } else {
                mTrackingType = ETrackingType.TARGET;
            }
        }
        else if (mDriverInputCodex.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            mTrackingType = ETrackingType.BALL;
        }
        else if (mDriverInputCodex.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            mTrackingType = ETrackingType.BALL_DUAL;
        }
        else if (mDriverInputCodex.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            mTrackingType = ETrackingType.BALL_TRI;
        }
        else {
            mTrackingType = ETrackingType.NONE;
            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if(!mTrackingType.equals(mLastTrackingType) && !mTrackingType.equals(ETrackingType.NONE)) {
            mLog.error("Requesting command start");
            mLog.error("Stopping teleop command queue");
            mTeleopCommandManager.stopRunningCommands(pNow);
            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
        mLastTrackingType = mTrackingType;
    }


    @Override
    public void shutdown(double pNow) {

    }

    @Override
    public double getThrottle() {
        if(mData.driverinput.isSet(InputMap.DRIVER.THROTTLE_AXIS)) {
            return -mData.driverinput.get(InputMap.DRIVER.THROTTLE_AXIS);
        } else {
            return 0.0;
        }
    }

    @Override
    public double getTurn() {
        if(mData.driverinput.isSet(InputMap.DRIVER.TURN_AXIS)) {
            return mData.driverinput.get(InputMap.DRIVER.TURN_AXIS);
        } else {
            return 0.0;
        }
    }
}
