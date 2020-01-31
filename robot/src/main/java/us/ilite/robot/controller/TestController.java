package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Field2020;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveMessage;
import us.ilite.robot.modules.Limelight;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.config.InputMap.DRIVER.*;

public class TestController extends AbstractController {

    private ILog mLog = Logger.createLog(TestController.class);
    private Double mLastTrackingType;
    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private double mLimelightZoomThreshold = 7.0;

    public TestController() { ;
    }

    public void update(double pNow) {
        updateLimelightTargetLock();
        updateDrivetrain(pNow);
    }

    public void updateLimelightTargetLock() {
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                SmartDashboard.putNumber("Distance to Target", Robot.DATA.limelight.get(ELimelightData.calcDistToTarget));
            }
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.TARGET.ordinal() );
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.ty) != null) {
                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.tx)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.TARGET_ZOOM.ordinal());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.TARGET.ordinal() );
                }
            } else {
                   Robot.DATA.selectedTarget.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.TARGET.ordinal());
            }
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.BALL.ordinal());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.BALL_DUAL.ordinal());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double) Field2020.FieldElement.BALL_TRI.ordinal());
        }
        else {
                Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE, (double)Limelight.NONE.id());
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if (!(Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal()).equals(mLastTrackingType) )
                && !(Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal()) == Limelight.NONE.id())) {
                mLog.error("Requesting command start");
                mLog.error("Stopping teleop command queue");
//            mTeleopCommandManager.stopRunningCommands(pNow);
//            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
        mLastTrackingType =  Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE.ordinal());
    }

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        if (throttle == 0.0 && rotate != 0.0) {
            throttle += 0.03;
        }
        var d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
        throttle = d.getThrottle();
        rotate = d.getTurn();
        if (db.driverinput.isSet(SUB_WARP_AXIS) && db.driverinput.get(SUB_WARP_AXIS) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
            throttle *= Settings.Input.kSnailModePercentThrottleReduction;
            rotate *= Settings.Input.kSnailModePercentRotateReduction;
        }
        db.drivetrain.set(THROTTLE, throttle);
        db.drivetrain.set(TURN, rotate);
    }

}