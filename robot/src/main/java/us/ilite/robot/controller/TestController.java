package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.DriverInput;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.LimelightTargetLock;
import us.ilite.robot.modules.CommandManager;
import us.ilite.robot.modules.FlywheelPrototype;
import us.ilite.robot.modules.Limelight;

public class TestController extends AbstractController {

    private ILog mLog = Logger.createLog(DriverInput.class);

    private Limelight mLimelight;
    private ETrackingType mTrackingType;
    private ETrackingType mLastTrackingType = null;

    private CommandManager mTestCommandManager;

    private double mLimelightZoomThreshold = 7.0;

    public TestController(Limelight pLimelight) {
        mLimelight = pLimelight;
    }

    public void update(double pNow) {
        updateLimelightTargetLock();
    }
    public void updateLimelightTargetLock(){
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)){
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                SmartDashboard.putNumber("Distance to Target", mLimelight.calcTargetDistance(72));
            }
            mTrackingType = ETrackingType.TARGET;
        }
        else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)){
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.tx)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.LIMELIGHT_STATE , (double) mTrackingType.ordinal());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.LIMELIGHT_STATE , (double) mTrackingType.ordinal());
                }
            } else {
             //   Robot.DATA.selectedTarget.set(E);
            }
        }
        else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            mTrackingType = ETrackingType.BALL;
        }
        else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            mTrackingType = ETrackingType.BALL_DUAL;
        }
        else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            mTrackingType = ETrackingType.BALL_TRI;
        }
        else {
            mTrackingType = ETrackingType.NONE;
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if(!mTrackingType.equals(mLastTrackingType) && !mTrackingType.equals(ETrackingType.NONE)) {
            mLog.error("Requesting command start");
            mLog.error("Stopping teleop command queue");
//            mTeleopCommandManager.stopRunningCommands(pNow);
//            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
        mLastTrackingType = mTrackingType;
    }
}
