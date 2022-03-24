package us.ilite.robot.controller;

import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.*;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.EFeederData.*;


public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private static TeleopController INSTANCE;

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
        updateCargo();
        updateHangerMotors();
        updateHangerPneumatics();
    }

    private void updateHangerMotors() {
        db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);

        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0.4);
            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, -0.4);
            } else if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, -90);
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
            }
            else if (db.operatorinput.isSet(InputMap.HANGER.HIGH_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, 90);
            }
            else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, 286);
            } else if (db.operatorinput.isSet(InputMap.HANGER.BALANCE_CLIMBER)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, 250);
            }
            else {
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0);
            }
        }
    }

    private void updateHangerPneumatics() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_DOUBLE)) {
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
            } if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_DOUBLE)) {
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
            }

            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_SINGLE)) {
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
            } if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_SINGLE)) {
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
            }
        }
    }

    private void updateCargo() {
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
                fireCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
                intakeCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
                placeCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
                reverseCargo();
            } else {
                db.feeder.set(SET_FEEDER_pct, 0d);
                db.intake.set(DESIRED_ROLLER_pct, 0d);
            }
        }
    }

    //TODO Confirm if we need these methods or not
//    private void updateIntake() {
//        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
//                db.intake.set(ARM_STATE, Enums.EArmState.DEFAULT);
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
//                db.intake.set(ARM_STATE, Enums.EArmState.RETRACT);
//            }
//        }
//    }
//
//    private void updateRollers() {
//        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS)) {
//                db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
//                db.intake.set(DESIRED_ROLLER_pct, -1.0);
//            }
//        }
//    }
}
