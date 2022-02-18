package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.DigitalBeamSensor;

import static us.ilite.common.types.EIntakeData.*;


public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private static TeleopController INSTANCE;
    private XorLatch mTurretReverseHome = new XorLatch();
    private boolean isBallAdded = false;
    private boolean isBallOut = false;
    private int numBalls = 0;

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
        clock.report("updateCargo", t->updateCargo());
        clock.report("updateCargo", t->updateFeeder());
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

    private void updateCargo() {
        //If not max balls and button down, bring arm down and start intaking
//        if(db.feeder.get(EFeederData.NUM_BALLS) < 2) {
        SmartDashboard.putBoolean("Button pressed: ", db.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS));
        SmartDashboard.putBoolean("isballadded: ", isBallAdded);
            if (db.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS)) { //left trigger
//                db.cargo.set(REV_PNEUMATIC_STATE, 0d);
//                db.cargo.set(FWD_PNEUMATIC_STATE, 1d);
                db.cargo.set(SET_ROLLER_VEL_ft_s, Math.max(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s), db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)) + 1000);
                //If beam breaker is broken, add one ball
                if (db.feeder.get(EFeederData.ENTRY_BEAM) == 1d && !isBallAdded) {
//                    db.feeder.set(EFeederData.NUM_BALLS, (db.feeder.get(EFeederData.NUM_BALLS) + 1d));
                    numBalls++;
                    isBallAdded = true;
                } else if (isBallAdded && db.feeder.get(EFeederData.ENTRY_BEAM) == 0d) {
                    isBallAdded = false;
                }
                //TODO need to add indexing
            }
            else {
//                db.cargo.set(FWD_PNEUMATIC_STATE, 0d);
//                db.cargo.set(REV_PNEUMATIC_STATE, 1d);
                db.cargo.set(SET_ROLLER_VEL_ft_s, 0);
            }
            db.feeder.set(EFeederData.NUM_BALLS, numBalls);
//        }

        //Reverse intake
        if(db.driverinput.isSet(ELogitech310.R_BTN)) { //r button
            if(db.feeder.get(EFeederData.NUM_BALLS) == 0d) {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0);
            }
            else {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, -0.2);
            }
        }
        else {
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0);
        }

    }

    private void updateFeeder() {
        //Shoot balls based off how many balls are in robot
        if(db.driverinput.isSet(ELogitech310.Y_BTN)) { // y button
            if(db.feeder.get(EFeederData.NUM_BALLS) >= 1d) {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
                if(db.feeder.get(EFeederData.EXIT_BEAM) == 1d) {
                    isBallOut = true;
                }
                else if (db.feeder.get(EFeederData.EXIT_BEAM) == 0d && isBallOut) {
                    numBalls--;
                    isBallOut = false;
                }
            }
            else {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0);
            }
        }
        db.feeder.set(EFeederData.NUM_BALLS, numBalls);
    }
}
