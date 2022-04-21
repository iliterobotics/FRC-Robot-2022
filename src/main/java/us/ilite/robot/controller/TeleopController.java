package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.EFeederData.*;


public class TeleopController extends BaseManualController {

    private static TeleopController INSTANCE;
    private boolean mPressed = false, mPrevPressed = false;

    private Timer mClimbTimer;

    public static TeleopController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
        mClimbTimer = new Timer();
        mClimbTimer.reset();
        mClimbTimer.start();
    }

    @Override
    protected void updateImpl() {
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        super.updateDrivetrain();
        updateCargo();
        super.updateBallCount();
        if (Robot.CLIMB_MODE.equals("WCMP")) {
            if (db.operatorinput.isSet(ELogitech310.START)) {
                updateHangerManual();
                mLastRungState = Enums.ERungState.NULL;
            } else {
                updateRungState();
            }
        } else {
            //Add in methods from DCMP
        }

        updateIntake();
        updateTargetLock();
    }

    private void updateTargetLock() {
        if (Robot.mode() == EMatchMode.TELEOPERATED) {
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
        } else {
            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG1);
                } else {
                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG2);
                }
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
            } else {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
            }
        }
    }
    private void updateHangerManual() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0.45);
            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, -0.45);
            } else {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
                db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0.0);
            }
            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_DOUBLE)) {
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
            }
            if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_DOUBLE)) {
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
            }
            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_SINGLE)) {
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
            }
            if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_SINGLE)) {
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
            }
        } else {
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
            db.climber.set(EClimberModuleData.DESIRED_VEL_pct, 0.0);
        }
    }
    private boolean mBalanced = false;
//    private void updateAutomaticHanger() {
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            double angle = db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg);
//            if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
//                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.MID.getAngle());
//            } else if (db.operatorinput.isSet(InputMap.HANGER.HIGH_RUNG)) {
//                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.HIGH.getAngle());
//                if (climberWithinTolerance(1, angle, Enums.EClimberAngle.HIGH)) {
//                    db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                }
//            }
//            //Everything works up to this point
//            else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
//                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                if (!mBalanced) {
//                    db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
//                } else {
//                    db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
//                }
//                if (angle <= Enums.EClimberAngle.BALANCED.getAngle()) {
//                    mBalanced = true;
//                }
//                if (angle >= Enums.EClimberAngle.HIGH.getAngle()) {
//                    db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                } else {
//                    db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//                }
//            }
//        }
//    }

    private Enums.ERungState mLastRungState = Enums.ERungState.NULL;
    protected void updateRungState() {
        Enums.ERungState newState = mLastRungState;
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {

            double angle = db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg);
            // Determine Rung State
            switch (mLastRungState) {
                case NULL:
                    if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
                        newState = Enums.ERungState.GROUND;
                    }
                    break;
                case GROUND:
                    if (db.operatorinput.isSet(InputMap.HANGER.GRAB_MID)) {
                        newState = Enums.ERungState.GO_TO_HIGH_BAR;
                    } else {
                        newState = Enums.ERungState.GROUND;
                    }
                    break;
                case GO_TO_HIGH_BAR:
                    if (climberWithinTolerance(1d, angle, Enums.EClimberAngle.HIGH)) {
                        newState = Enums.ERungState.GRAB_HIGH_BAR;
                    } else {
                        newState = Enums.ERungState.GO_TO_HIGH_BAR;
                    }
                    break;
                case GRAB_HIGH_BAR:
                    if (db.operatorinput.isSet(InputMap.HANGER.CONFIRM_CLAMPED_ON_HIGH_BAR)) {
                        newState = Enums.ERungState.BALANCING;
                    } else {
                        newState = Enums.ERungState.GRAB_HIGH_BAR;
                    }
                    break;
                case BALANCING:
                    if (climberWithinTolerance(2, angle, Enums.EClimberAngle.BALANCED)) {
                        newState = Enums.ERungState.RELEASING_MID;
                    } else {
                        newState = Enums.ERungState.BALANCING;
                    }
                    break;
                    // If angle = balanced angle, then go to RELEASE_MID state
                case RELEASING_MID:
                    //Instantaneously move to next stage to traversal once released
                    newState = Enums.ERungState.MOVE_TO_TRAVERSAL;
                    break;
                case MOVE_TO_TRAVERSAL:
                    if (climberWithinTolerance(5, angle, Enums.EClimberAngle.TRAVERSAL)) {
                        newState = Enums.ERungState.GRAB_TRAVERSAL;
                    }
                    break;
                case GRAB_TRAVERSAL:
                    if (db.operatorinput.isSet(InputMap.HANGER.CONFIRM_CLAMPED_ON_TRAVERSAL_RELEASE_HIGH)) {
                        newState = Enums.ERungState.RELEASE_HIGH;
                    }
                    else {
                        newState = Enums.ERungState.GRAB_TRAVERSAL;
                    }
                    break;
                case RELEASE_HIGH:
                    setIntakeArmEnabled(true);
                    newState = Enums.ERungState.RELEASE_HIGH;
                    break;
                default:
                    newState = Enums.ERungState.NULL;

            }
        }

        // Set outputs based upon new state
        switch (newState) {
            case GROUND:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.MID.getAngle());
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
                break;
            case GO_TO_HIGH_BAR:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.HIGH.getAngle());
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
                break;
            case GRAB_HIGH_BAR:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.HIGH.getAngle());
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                break;
            case BALANCING:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                break;
            case RELEASING_MID:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
                break;
            case MOVE_TO_TRAVERSAL:
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
                break;
            case GRAB_TRAVERSAL:
                setIntakeArmEnabled(true);
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                break;
            case RELEASE_HIGH:
                setIntakeArmEnabled(true);
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
                db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
                db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                break;
            case FINAL_LIFT:
                updateHangerManual();
                break;
        }
        mLastRungState = newState;
        db.climber.set(EClimberModuleData.RUNG_STATE, newState);
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
