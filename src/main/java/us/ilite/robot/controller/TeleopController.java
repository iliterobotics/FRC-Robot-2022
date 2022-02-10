package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.Field2020;
import us.ilite.common.config.InputMap;
import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.modules.Limelight;

import static us.ilite.common.types.EIntakeData.*;

public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private static TeleopController INSTANCE;
    private Enums.FlywheelSpeeds currentState = Enums.FlywheelSpeeds.CLOSE;
    private XorLatch mTurretReverseHome = new XorLatch();

    public static TeleopController getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
        db.registerAllWithShuffleboard();
    }

    @Override
    protected void updateImpl() {
        db.registerAllWithShuffleboard();
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        //updateLimelightTargetLock(); //waiting for merge to master
        super.updateDrivetrain();
//        updateHanger(); //not integrated yet
    }

//    private void updateHanger() {
//        if (db.operatorinput.get(InputMap.OPERATOR.BEGIN_HANG) >= 0.5 && db.driverinput.isSet(InputMap.DRIVER.HANGER_LOCK)) {
//            db.hanger.set(EHangerModuleData.SET_pct, (db.operatorinput.get(ELogitech310.LEFT_Y_AXIS)));
//        }else{
//            db.hanger.set(EHangerModuleData.SET_pct, 0.0);
//        }
//
//    }

//    public void updateLimelightTargetLock() {
//        if (DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
//            if (DATA.selectedTarget.isSet(ELimelightData.TY)) {
//                SmartDashboard.putNumber("Distance to Target", DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
//            }
//            DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//        } else if (DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
//            if (DATA.selectedTarget.isSet(ELimelightData.TY)) {
//                if (Math.abs(DATA.selectedTarget.get(ELimelightData.TX)) < mLimelightZoomThreshold) {
//                    DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET_ZOOM.id());
//                } else {
//                    DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//                }
//            } else {
//                DATA.selectedTarget.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//            }
//        } else if (DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
//            DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL.id());
//        } else if (DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
//            DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_DUAL.id());
//            DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_TRI.id());
//        } else {
//            DATA.limelight.set(ELimelightData.TARGET_ID, (double) Limelight.NONE.id());
//        }
//        if ((DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) != (mLastTrackingType))
//                && !(DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) == Limelight.NONE.id())) {
//            mLog.error("Requesting command start");
//            mLog.error("Stopping teleop command queue");
//        }
//        mLastTrackingType = DATA.limelight.get(ELimelightData.TARGET_ID.ordinal());
//    }


}
