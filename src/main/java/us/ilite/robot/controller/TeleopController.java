package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.EFeederData.*;


public class TeleopController extends BaseManualController {

    private static TeleopController INSTANCE;
    private boolean mPressed = false, mPrevPressed = false;

    public static TeleopController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
    }

    @Override
    protected void updateImpl() {
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        super.updateDrivetrain();
        updateCargo();
        super.updateBallCount();
        updateHangerMotors();
        updateHangerPneumatics();
        updateIntake();
        updateTargetLock();
    }

    private void updateTargetLock() {
        if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
            if (db.limelight.isSet(ELimelightData.TV)) {
                setLED(Enums.LEDColorMode.GREEN, Enums.LEDState.SOLID);
            } else {
                setLED(Enums.LEDColorMode.RED, Enums.LEDState.SOLID);
            }
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.BLUE_BALL.id());
            } else {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.RED_BALL.id());
            }
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
        } else {
            db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
        }
    }

    private void updateHangerMotors() {
        db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);

        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0.45);
            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, -0.45);
            } else if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, -90);
//                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
            }
            else if (db.operatorinput.isSet(InputMap.HANGER.HIGH_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, 90);
            }
            else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, 287.5);
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
        db.feeder.set(ADJUSTABLE_FEEDER_PCT, mFeederFireSpeed);
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            if (db.operatorinput.isSet(InputMap.OPERATOR.INCREASE_FEEDER_SPEED) ||
                    db.operatorinput.isSet(InputMap.OPERATOR.DECREASE_FEEDER_SPEED)) {
                mPressed = true;
            } else {
                mPressed = false;
            }
            if (mPressed && !mPrevPressed && db.operatorinput.isSet(InputMap.OPERATOR.INCREASE_FEEDER_SPEED)) {
                mFeederFireSpeed += 500d;
            } else if (mPressed && !mPrevPressed && db.operatorinput.isSet(InputMap.OPERATOR.DECREASE_FEEDER_SPEED)) {
                mFeederFireSpeed -= 500d;
            }
            if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
                fireCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                intakeCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                placeCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                reverseCargo();
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.STAGE_BALLS)) {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                stageBalls();
            } else {
                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
                mBallsShot = 0;
                mShotTimer.reset();
                db.feeder.set(SET_FEEDER_pct, 0d);
                db.intake.set(DESIRED_ROLLER_pct, 0d);
            }
        }
        mPrevPressed = mPressed;
    }
    private void updateIntake() {
        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
                setIntakeArmEnabled(true);
            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
                setIntakeArmEnabled(false);
            }
        }
    }

}
