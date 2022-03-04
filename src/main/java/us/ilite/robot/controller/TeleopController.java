package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.*;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.EFeederData.*;


public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private static TeleopController INSTANCE;

    private boolean mResetCount = false;
    private boolean mPrevResetCount = false;
    private boolean mAddBalls = false;
    private boolean mPrevAddBalls = false;

    public static TeleopController getInstance() {
        if (INSTANCE == null) {
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
        super.updateDrivetrain();
        updateIntake();
        updateCargo();
        updateHangerMotors();
        updateHangerPneumatics();
    }

    private void updateHangerMotors() {
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.HANGER.POSITION_LOCK)) {
//                // Set the state to position mode
//            } else if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_FWD)) {
//                db.hanger.set(EHangerModuleData.SET_pct, 0.75);
//            } else if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_REV)) {
//                db.hanger.set(EHangerModuleData.SET_pct, -0.75);
//            } else {
//                db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.WHITE);
//            }
//
//            if (db.operatorinput.isSet(InputMap.HANGER.LOCK_FWD)) {
//                // Lock or unlock the first solenoid
//                db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.BLUE);
//            } else if (db.operatorinput.isSet(InputMap.HANGER.LOCK_REV)) {
//                // Lock or unlock the second solenoid
//                db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.RED);
//            }
//        }
//        if(db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
////            if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_FWD_FULL)) {
////                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.PERCENT_OUTPUT);
////                db.hanger.set(EHangerModuleData.L_SET_pct, 1);
////                db.hanger.set(EHangerModuleData.R_SET_pct, 1);
////            } else if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_FWD_HALF_SPEED)) {
////                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.PERCENT_OUTPUT);
////                db.hanger.set(EHangerModuleData.L_SET_pct, 0.5);
////                db.hanger.set(EHangerModuleData.R_SET_pct, 0.5);
////            }
//
////            else if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_REV_FULL)) {
////                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.PERCENT_OUTPUT);
////                db.hanger.set(EHangerModuleData.L_SET_pct, -1);
////                db.hanger.set(EHangerModuleData.R_SET_pct, -1);
////            } else if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_REV_HALF_SPEED)) {
////                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.PERCENT_OUTPUT);
////                db.hanger.set(EHangerModuleData.L_SET_pct, -0.5);
////                db.hanger.set(EHangerModuleData.R_SET_pct, -0.5);
////            }
//            if (db.operatorinput.isSet(InputMap.HANGER.MANUAL_REV_SLOW)) {
//                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.PERCENT_OUTPUT);
//                db.hanger.set(EHangerModuleData.L_SET_pct, -0.2);
//                db.hanger.set(EHangerModuleData.R_SET_pct, -0.2);
//            } else {
//                db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.DEFAULT);
//            }
//        }

        if (db.driverinput.isSet(InputMap.DRIVER.MANUAL_FWD_SLOW)) {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT.ordinal());
            db.climber.set(EClimberModuleData.L_SET_pct, 0.4);
            db.climber.set(EClimberModuleData.R_SET_pct, 0.4);
        }
        else if (db.driverinput.isSet(InputMap.DRIVER.MANUAL_REV_SLOW)) {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT.ordinal());
            db.climber.set(EClimberModuleData.L_SET_pct, -0.4);
            db.climber.set(EClimberModuleData.R_SET_pct, -0.4);
        }
        else {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT.ordinal());
            db.climber.set(EClimberModuleData.L_SET_pct, 0);
            db.climber.set(EClimberModuleData.R_SET_pct, 0);
        }
    }

    private void updateHangerPneumatics() {
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {

        if (db.operatorinput.isSet(InputMap.OPERATOR.TOP_CLAMPED)) {
            db.climber.set(EClimberModuleData.IS_A_CLAMPED, 1.0);
        } if (db.operatorinput.isSet(InputMap.OPERATOR.TOP_RELEASED)) {
            db.climber.set(EClimberModuleData.IS_A_CLAMPED, 2.0);
        }

        if (db.operatorinput.isSet(InputMap.OPERATOR.BOTTOM_CLAMPED)) {
            db.climber.set(EClimberModuleData.IS_B_CLAMPED, 1.0);
        } if (db.operatorinput.isSet(InputMap.OPERATOR.BOTTOM_RELEASED)) {
            db.climber.set(EClimberModuleData.IS_B_CLAMPED, 2.0);
        }

//        }
    }

    private void updateIntake() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
            db.intake.set(ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
            db.intake.set(ARM_STATE, Enums.EArmState.RETRACT);
        }
    }

    private void updateRollers() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_ROLLERS)) {
            db.intake.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.intake.set(DESIRED_pct, 1.0);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS)) {
            db.intake.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.intake.set(DESIRED_pct, -1.0);
        }
    }

//    private void updateCargo() {
//        db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
//        db.intake.set(EIntakeData.ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
//
//        if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
//            db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.RED);
//            fireCargo();
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
//            indexCargo();
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
//            placeCargo();
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
//            reverseCargo();
//        } else {
//            db.feeder.set(SET_FEEDER_pct, 0d);
//            db.intake.set(DESIRED_pct, 0d);
//        }
//    }

    private void updateCargo() { // Experimental way of incorporating ball count into indexing
//        db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//        db.intake.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
//
//        if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
//            db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.RED);
//            fireCargo();
//            mResetCount = true;
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
//            indexCargo();
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
//            placeCargo();
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
//            reverseCargo();
//            mResetCount = true;
//        } else {
//            db.feeder.set(SET_FEEDER_pct, 0d);
//            db.intake.set(DESIRED_pct, 0d);
//            mResetCount = false;
//        }
//
//        if (db.operatorinput.isSet(InputMap.OPERATOR.MANUAL_BALLS_UP)) {
//            mAddBalls = true;
//            if (!mPrevAddBalls) {
//                db.feeder.set(NUM_BALLS, db.feeder.get(NUM_BALLS) + 1);
//            }
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.MANUAL_BALLS_DOWN)) {
//            mAddBalls = true;
//            if (!mPrevAddBalls) {
//                db.feeder.set(NUM_BALLS, db.feeder.get(NUM_BALLS) - 1);
//            }
//        } else {
//            mAddBalls = false;
//        }
//
//        if (mPrevResetCount && !mResetCount) {
//            db.feeder.set(RESET_BALLS, 1d);
//        } else {
//            db.feeder.set(RESET_BALLS, 0d);
//        }
//
//        mPrevResetCount = mResetCount;
//        mPrevAddBalls = mAddBalls;
    }

    private void updateLimelightTargetLock() {
        if (db.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            db.limelight.set(ELimelightData.PIPELINE, Field2022.FieldElement.HUB_UPPER.pipeline());
            db.limelight.set(ELimelightData.LED_MODE, 1);
        } else {
            db.limelight.set(ELimelightData.PIPELINE, Field2022.FieldElement.NONE.pipeline());
            db.limelight.set(ELimelightData.LED_MODE, 0);
        }
    }

    private void updateFeeder() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
            db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(SET_FEEDER_pct, 1.0);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_FEEDER)) {
            db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(SET_FEEDER_pct, -1.0);
        }
    }
}
