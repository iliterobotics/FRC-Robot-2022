package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
        updateHangerManual();
        updateIntake();
        updateTargetLock();
        updateAutomaticHanger();
//        updateRungs(); // TODO Fix experimental balancing values and test timer and overall logic
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
    private void updateAutomaticHanger() {
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
            if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.MID.getAngle());
            } else if (db.operatorinput.isSet(InputMap.HANGER.HIGH_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.HIGH.getAngle());
                if (db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg) >= 90
                        && db.climber.get(EClimberModuleData.DESIRED_POS_deg) <= 92) {
                    db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                }
            } else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
                db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
                if (!mBalanced) {
                    db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
                } else {
                    db.climber.set(EClimberModuleData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
                }
                if (db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg) <= Enums.EClimberAngle.BALANCED.getAngle()) {
                    mBalanced = true;
                }
                if (db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg) >= Enums.EClimberAngle.HIGH.getAngle()) {
                    db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                } else {
                    db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
                }
            }
        }
    }

    private void updateRungs() {
        double angle = db.climber.get(EClimberModuleData.ACTUAL_POSITION_deg);
        if (Math.abs(angle - Enums.EClimberAngle.MID.getAngle()) < 2.0) {
            db.climber.set(EClimberModuleData.CURRENT_RUNG, Enums.EClimberAngle.MID);
        } else if (Math.abs(angle - Enums.EClimberAngle.HIGH.getAngle()) < 2.0) {
            db.climber.set(EClimberModuleData.CURRENT_RUNG, Enums.EClimberAngle.HIGH);
        } else if (Math.abs(angle - Enums.EClimberAngle.TRAVERSAL.getAngle()) < 2.0) {
            db.climber.set(EClimberModuleData.CURRENT_RUNG, Enums.EClimberAngle.TRAVERSAL);
        }
        Enums.EClimberAngle desiredRung = db.climber.get(EClimberModuleData.DESIRED_RUNG, Enums.EClimberAngle.class);
        int stage = (int)db.climber.get(EClimberModuleData.STAGE);
        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) { // If DRIVER is holding [START]
            db.climber.set(EClimberModuleData.HANGER_STATE, Enums.EClimberMode.POSITION);
            if (db.operatorinput.isSet(InputMap.HANGER.CLIMB_TO_NEXT_RUNG)) { // If OPERATOR is holding [RB]
                if (stage == 0) { // Heading towards Mid-Bar
                    desiredRung = Enums.EClimberAngle.MID;
                    if (climberWithinTolerance(2.0)) { // If the climber error is less than 2 degrees
                        db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                        db.climber.set(EClimberModuleData.STAGE, 1);
                        mClimbTimer.reset();
                    }
                } else if (stage == 1) { // Heading towards High-Bar
                    desiredRung = Enums.EClimberAngle.HIGH;
                    if (climberWithinTolerance(2.0)) { // If the climber error is less than 2 degrees
                        db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
                        db.climber.set(EClimberModuleData.STAGE, 2);
                        mClimbTimer.reset();
                    }
                } else if (stage == 2) { // Balancing out at High-Bar
                    desiredRung = Enums.EClimberAngle.BALANCED;
                    if (climberWithinTolerance(2.0)) { // If the climber error is less than 2 degrees
                        db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
                        db.climber.set(EClimberModuleData.STAGE, 3);
                        mClimbTimer.reset();
                    }
                } else if (stage == 3) { // Heading towards Traversal-Bar
                    desiredRung = Enums.EClimberAngle.TRAVERSAL;
                    if (climberWithinTolerance(2.0)) { // If the climber error is less than 2 degrees
                        db.climber.set(EClimberModuleData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
                        db.climber.set(EClimberModuleData.STAGE, 4);
                        mClimbTimer.reset();
                    }
                } else if (stage == 4) { // Score the climb
                    desiredRung = Enums.EClimberAngle.SCORE;
                    db.climber.set(EClimberModuleData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
                }

                if (mClimbTimer.get() > 0.25) { // Give a quarter of a second for pneumatics to properly actuate before moving on the the next rung
                    db.climber.set(EClimberModuleData.DESIRED_POS_deg, desiredRung.getAngle());
                }
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
