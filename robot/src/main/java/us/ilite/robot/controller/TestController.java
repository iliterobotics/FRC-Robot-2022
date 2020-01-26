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
    private Double mLastTrackingType;

    private CommandManager mTestCommandManager;

    private double mLimelightZoomThreshold = 7.0;

    public TestController(Limelight pLimelight) {
        mLimelight = pLimelight;
    }

    public void update(double pNow) {
        updateLimelightTargetLock();
    }

    public void updateLimelightTargetLock() {
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                SmartDashboard.putNumber("Distance to Target", mLimelight.calcTargetDistance(72));
            }
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.TARGET.ordinal() );
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.tx)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.TARGET_ZOOM.ordinal());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.TARGET.ordinal() );
                }
            } else {
                //   Robot.DATA.selectedTarget.set(E);
            }
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.BALL.ordinal());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.BALL_DUAL.ordinal());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.BALL_TRI.ordinal());
        }
        else {
                Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) ETrackingType.NONE.ordinal());
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if (!(Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal()).equals(mLastTrackingType) )
                && !(Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal()) == ETrackingType.NONE.ordinal())) {
                mLog.error("Requesting command start");
                mLog.error("Stopping teleop command queue");
//            mTeleopCommandManager.stopRunningCommands(pNow);
//            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
            mLastTrackingType =  Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal());
        }
    }

