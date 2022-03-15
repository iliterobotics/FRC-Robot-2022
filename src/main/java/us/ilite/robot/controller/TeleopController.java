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

    private double mCurrentDoubleClamp = 0d;
    private double mCurrentSingleClamp = 0d;

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
        updateRollers();
        updateFeeder();
        updateHangerMotors();
        updateHangerPneumatics();
    }

    private void updateHangerMotors() {
        if (db.operatorinput.isSet(InputMap.HANGER.POSITION_LOCK)) {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
            db.climber.set(EClimberModuleData.DESIRED_POS_deg, 90);
        } else {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.VELOCITY);
        }

        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_rpm, 10);
            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_rpm, -10);
            } else {
                db.climber.set(EClimberModuleData.DESIRED_VEL_rpm, 0);
            }

            if (db.operatorinput.isSet(InputMap.HANGER.SET_COAST)) {
                db.climber.set(EClimberModuleData.IS_COAST, 1d);
            }
        }
    }

    private void updateHangerPneumatics() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_DOUBLE)) {
                db.climber.set(EClimberModuleData.DOUBLE_CLAMPED, 1d);
                mCurrentDoubleClamp = 1d;
            } else if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_DOUBLE)) {
                db.climber.set(EClimberModuleData.DOUBLE_CLAMPED, 0d);
                mCurrentDoubleClamp = 0d;
            } else {
                db.climber.set(EClimberModuleData.DOUBLE_CLAMPED, mCurrentDoubleClamp);
            }

            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_SINGLE)) {
                db.climber.set(EClimberModuleData.SINGLE_CLAMPED, 1d);
                mCurrentSingleClamp = 1d;
            } else if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_SINGLE)) {
                db.climber.set(EClimberModuleData.SINGLE_CLAMPED, 0d);
                mCurrentSingleClamp = 0d;
            } else {
                db.climber.set(EClimberModuleData.SINGLE_CLAMPED, mCurrentDoubleClamp);
            }
        }
    }

    private void updateIntake() {
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
                db.intake.set(ARM_STATE, Enums.EArmState.DEFAULT);
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
                db.intake.set(ARM_STATE, Enums.EArmState.RETRACT);
            }
        }
    }

    private void updateRollers() {
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_ROLLERS)) {
                db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
                db.intake.set(DESIRED_pct, 1.0);
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS)) {
                db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
                db.intake.set(DESIRED_pct, -1.0);
            }
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
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                db.feeder.set(SET_FEEDER_pct, 1.0);
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_FEEDER)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                db.feeder.set(SET_FEEDER_pct, -1.0);
            }
        }
    }
}
