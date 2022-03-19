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
        updateRollers();
//        updateFeeder();
        updateCargo();
        updateHangerMotors();
        updateHangerPneumatics();
    }

    private void updateHangerMotors() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.OPERATOR.MANUAL_FWD_SLOW)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT.ordinal());
                db.climber.set(EClimberModuleData.L_SET_pct, 0.4);
                db.climber.set(EClimberModuleData.R_SET_pct, 0.4);
            }
            else if (db.operatorinput.isSet(InputMap.OPERATOR.MANUAL_REV_SLOW)) {
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

        if (db.driverinput.isSet(InputMap.OPERATOR.SET_COAST)) {
            db.climber.set(EClimberModuleData.IS_COAST, 1d);
        }
    }

    private void updateHangerPneumatics() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
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
            if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS)) {
                db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
                db.intake.set(DESIRED_ROLLER_pct, -1.0);
            }
        }
    }

    private void updateCargo() {
        db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
        db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
        if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
            fireCargo();
            mResetCount = true;
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
            db.intake.set(DESIRED_ROLLER_pct, 1.0);
            db.intake.set(ARM_STATE, Enums.EArmState.DEFAULT);
            indexCargo();
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
            placeCargo();
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
            reverseCargo();
            mResetCount = true;
        } else {
            db.feeder.set(SET_FEEDER_pct, 0d);
            db.intake.set(DESIRED_ROLLER_pct, 0d);
            mResetCount = false;
        }
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
