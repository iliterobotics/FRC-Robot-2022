package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.util.Color;
import com.flybotix.hfr.util.lang.EnumUtils;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EColorData;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Field2020;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.*;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EHangerModuleData;

import static us.ilite.common.config.InputMap.OPERATOR.FIRE_POWER_CELLS;
import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;
import static us.ilite.robot.modules.DriveModule.kDriveNEOVelocityFactor;

import java.util.List;

public class TestController extends BaseManualController {

    private ILog mLog = Logger.createLog(TestController.class);
    private Double mLastTrackingType = 0d;

    private double mLimelightZoomThreshold = 7.0;

    private PowerCellModule.EIntakeState mIntakeState;
    private PowerCellModule.EArmState mArmState;
    private double mPreviousTime;
    private double mGoalBeamCountBroken = 0;
    private boolean crossedEntry = false;
    private Boolean allBeamsBroken = false;


    private static TestController INSTANCE;

    public static TestController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TestController();
        }
        return INSTANCE;
    }

    private TestController() {
        for (String key : db.mMappedCodex.keySet()) {
            ShuffleboardTab tab = Shuffleboard.getTab("TEST-" + key);
            List<Enum<?>> enums = EnumUtils.getEnums(db.mMappedCodex.get(key).meta().getEnum(), true);
            enums.stream().forEach(
                    e -> {
                        tab.addNumber(e.name(), () -> {
                            if (db.mMappedCodex.get(key).isSet(e)) {
                                return db.mMappedCodex.get(key).get(e);
                            } else {
                                return 0d;
                            }
                        });
                    }
            );
        }
    }

    private double mMaxSpeed = 0.0;
    private double mMaxYaw = 0.0;
    protected void updateImpl(double pNow) {
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
//        Robot.CLOCK.report("updateLimelightTargetLock", t -> updateLimelightTargetLock());
//        Robot.CLOCK.report("updateDrivetrain", t -> updateDrivetrain(pNow));
//        Robot.CLOCK.report("updateFlywheel", t -> updateFlywheel(pNow));
        Robot.CLOCK.report("updateIntake", t -> updatePowerCells(pNow));
//        Robot.CLOCK.report("updateHanger", t -> updateHanger(pNow));
//        Robot.CLOCK.report("updateDJBooth", t -> updateDJBooth(pNow));
//        updateArm(pNow);

        double spd = Math.max(db.drivetrain.get(R_ACTUAL_VEL_FT_s), db.drivetrain.get(L_ACTUAL_VEL_FT_s));
        mMaxSpeed = Math.max(mMaxSpeed, spd);
        SmartDashboard.putNumber("Max Robot Speed (ft/s)", mMaxSpeed);
        SmartDashboard.putNumber("Max Drive RPM", mMaxSpeed / kDriveNEOVelocityFactor);

        mMaxYaw = Math.max(mMaxYaw, db.imu.get(EGyro.YAW_OMEGA_DEGREES));
        SmartDashboard.putNumber("Max Robot Omega (deg/s)", mMaxYaw);
    }

    private void updateHanger(double pNow) {
        if (db.operatorinput.isSet(InputMap.OPERATOR.BEGIN_HANG)) {
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_POSITION, 17.0);
        } else {
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_POSITION, 0.0);
        }

    }

    void updateFlywheel(double pNow) {
//        if (!db.driverinput.isSet(InputMap.DRIVER.FLYWHEEL_AXIS) ) {
//            mTurretMode = FlywheelModule.ETurretMode.LIMELIGHT;
//            mHoodState = FlywheelModule.EHoodState.ADJUSTABLE;
//            mAcceleratorState = FlywheelModule.EAcceleratorState.FEED;
//            mShooterState = FlywheelModule.EShooterState.SHOOT;
//        }
//        else {
//            mTurretMode = FlywheelModule.ETurretMode.GYRO;
//            mAcceleratorState = FlywheelModule.EAcceleratorState.STOP;
//            mShooterState = FlywheelModule.EShooterState.STOP;
//            mHoodState = FlywheelModule.EHoodState.STATIONARY;
//        }
//
//        switch(mAcceleratorState) {
//            case FEED: db.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, FlywheelModule.kAcceleratorTargetVelocity);
//                break;
//            case STOP: db.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, 0.0);
//                break;
//        }
//
//        switch(mTurretMode) {
//            case GYRO: db.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-2 * mTurretGyro.getCompassHeading(), pNow - mPreviousTime));
//                break;
//            case LIMELIGHT:
//                if ( db.limelight.isSet(ETargetingData.tx)) {
//                    db.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-10 * db.limelight.get(ETargetingData.tx), pNow - mPreviousTime));
//                }
//                break;
//        }
//
//        switch(mShooterState) {
//            case SHOOT:
//                if ( db.limelight.isSet(ETargetingData.ty)) {
//                    db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, mShooter.calcSpeedFromDistance(db.limelight.get(ETargetingData.calcDistToTarget)));
//                }
//                else {
//                    db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, mShooterPid.calculate(Settings.ShooterSystem.kShooterTargetVelocity, 0.5));
//                }
//                break;
//            case STOP: db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0.0);
//                break;
//        }
//
//        switch(mHoodState) {
//            case STATIONARY: db.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
//                break;
//            case ADJUSTABLE:
//                if (db.limelight.isSet(ETargetingData.ty)) {
//                    db.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, mShooter.calcAngleFromDistance(db.limelight.get(ETargetingData.calcDistToTarget), db.limelight.get(ETargetingData.ty)));
//                }
//                else {
//                    db.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
//                }
//        }
//        if ( db.attackoperatorinput.isSet(ELogitechAttack3.TRIGGER)) {
//            mAccelerator.set(ControlMode.PercentOutput, db.attackoperatorinput.get(ELogitechAttack3.TRIGGER));
//        }
//        mPreviousTime = pNow;
//        mLog.error("-------------------------------------------------------Flywheel Velocity: ", db.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
    }

    public void updateLimelightTargetLock() {
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            if (Robot.DATA.selectedTarget.isSet(ELimelightData.TY)) {
                SmartDashboard.putNumber("Distance to Target", Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
            }
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.selectedTarget.isSet(ELimelightData.TY)) {
                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.TX)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET_ZOOM.id());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
                }
            } else {
                Robot.DATA.selectedTarget.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
            }
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL.id());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_DUAL.id());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_TRI.id());
        } else {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Limelight.NONE.id());
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if ((Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) != (mLastTrackingType))
                && !(Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) == Limelight.NONE.id())) {
            mLog.error("Requesting command start");
            mLog.error("Stopping teleop command queue");
//            mTeleopCommandManager.stopRunningCommands(pNow);
//            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
        mLastTrackingType = Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal());
    }

    boolean tripped = false;
    private boolean lastExitBeam = false;

    protected void updatePowerCells(double pNow) {
        // Default to none
        db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.NONE);
        // Practice bot testing, min power
        // Power of 1.0 was waaaaay too fast
        if (db.operatorinput.isSet(ELogitech310.L_BTN)) {
            db.powercell.set(UNUSED, 0.2);
        } else {
            db.powercell.set(UNUSED, 0.0);
        }

        if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_ACTIVATE)) {
            setIntakeArmEnabled(pNow, true);
            crossedEntry = activateSerializer(pNow);
//            if (crossedEntry && !db.powercell.isSet(EPowerCellData.SECONDARY_BREAM_BREAKER)) {
//                db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, power);
//                db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, power);
//            } else if ( db.powercell.isSet(EPowerCellData.SECONDARY_BREAM_BREAKER) ) {
//                crossedEntry = false;
//            }

        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_REVERSE)) {
            db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.STOW);
            reverseSerializer(pNow);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_STOW)) {
            db.powercell.set(DESIRED_INTAKE_VELOCITY_FT_S, 0d);
            setIntakeArmEnabled(pNow, false);
            crossedEntry = activateSerializer(pNow);
        } else {
            // TODO - only enable once we have set the hold gains
//            db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.HOLD);
            db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.NONE);
        }
        if(db.operatorinput.isSet(FIRE_POWER_CELLS)) {
            boolean exit = db.powercell.isSet(EXIT_BEAM);
            if(!exit || tripped) {
                db.powercell.set(DESIRED_V_VELOCITY, 0.5);
                db.powercell.set(DESIRED_H_VELOCITY, 0.5);
            } else {
                if (exit && !tripped ) {
                    db.powercell.set(DESIRED_V_VELOCITY, -0.3);
                    db.powercell.set(DESIRED_H_VELOCITY, -0.3);
                } else if(exit && !lastExitBeam) {
                    tripped = true;
                }
            }
        } else {
            db.powercell.set(DESIRED_V_VELOCITY, 0.0);
            db.powercell.set(DESIRED_H_VELOCITY, 0.0);
            tripped = false;
            lastExitBeam = false;
        }


//        if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE)) {
////            db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
////            db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
////            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
//
////            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle()); TODO Commented cuz we don't have an arm
////            if (db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN) != db.powercell.get(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN)){
//                if (db.powercell.get(EPowerCellData.ALL_BEAMS_BROKEN) == 0.0) {
//                    db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, db.drivetrain.get(LEFT_VEL_IPS) + PowerCellModule.kDeltaIntakeVel);
//                } else {
//                    db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
//                    db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, PowerCellModule.EIntakeState.STOP.getPower());
//                }
////            }
//
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
//            db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY , PowerCellModule.EIntakeState.REVERSE.getPower());
//            db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY , PowerCellModule.EIntakeState.REVERSE.getPower());
//            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY , PowerCellModule.EIntakeState.REVERSE.getPower());
//            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle());
////
////            if (db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN) != db.powercell.get(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN)){
////                if ( db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN) < mGoalBeamCountBroken) {
////                    db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
////                    db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
////                    db.powercell.se   t(EPowerCellData.DESIRED_INTAKE_VELOCITY, PowerCellModule.EIntakeState.INTAKE.getPower());
////
////                } else {
////                    db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
////                    db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
////                    db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY, PowerCellModule.EIntakeState.STOP.getPower());
////                }
////            }
//
//        } else {
//            mIntakeState = PowerCellModule.EIntakeState.STOP;
//            mArmState = PowerCellModule.EArmState.DISENGAGED;
//            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle());
//            db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, mIntakeState.getPower());
//            db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, mIntakeState.getPower());
//            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, db.drivetrain.get(LEFT_VEL_IPS) + PowerCellModule.kDeltaIntakeVel);
//        }

    }

    void updateDJBooth(double pNow) {
        if (db.operatorinput.isSet(InputMap.OPERATOR.COLOR_POSITION)) {
            db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.POSITION.getPower());
            int i = (int) db.color.get(EColorData.SENSED_COLOR);
            DJSpinnerModule.EColorMatch m = DJSpinnerModule.EColorMatch.values()[i];
            Color DJ_COLOR = null;
            switch (db.recieveColorFmsRelay()) {
                case 'B':
                    DJ_COLOR = DJSpinnerModule.EColorMatch.BLUE.color;
                    break;
                case 'G':
                    DJ_COLOR = DJSpinnerModule.EColorMatch.GREEN.color;
                    break;
                case 'R':
                    DJ_COLOR = DJSpinnerModule.EColorMatch.RED.color;
                    break;
                case 'Y':
                    DJ_COLOR = DJSpinnerModule.EColorMatch.YELLOW.color;
                    break;
                default:
                    DJ_COLOR = null;
                    break;
            }
            if (m.color.equals(DJ_COLOR)) {
                //TODO stop using the module for the desired power
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.OFF.getPower());
            } else {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.POSITION.getPower());
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, DJSpinnerModule.EColorWheelState.POSITION.ordinal());
            }
        }
        else if ( db.operatorinput.isSet(InputMap.OPERATOR.COLOR_ROTATION)) {
            db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.ROTATION.getPower());
            if(db.color.get(EColorData.WHEEL_ROTATION_COUNT) >= DJSpinnerModule.TARGET_ROTATION_COUNT) {
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, DJSpinnerModule.EColorWheelState.OFF.ordinal());
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.OFF.getPower());
            } else {
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, DJSpinnerModule.EColorWheelState.ROTATION.ordinal());
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.ROTATION.getPower());
            }
        }

    }
}



//    void updateDJBooth() {
//        if ( db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL)) {
//            DJSpinnerModule.EColorMatch m =db.color.get(EColorData.SENSED_COLOR, DJSpinnerModule.EColorMatch.class);
//            if(m.color.equals(db.DJ_COLOR)) {
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.OFF.power);
//            } else {
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.POSITION.power);
//                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) DJSpinnerModule.EColorWheelState.POSITION.ordinal());
//            }
//        }
//


//
//
//        if ( db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL) &&
//                db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_ROTATION_CONTROL) ) {
//            db.color.set(EColorData.POSITION_CONTROL_INPUT, (double)EColorData.EInput.NEGATIVE.ordinal());
//            db.color.set(EColorData.ROTATION_CONTROL_INPUT, (double)EColorData.EInput.NEGATIVE.ordinal());
//        }
//        else if (db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL)) {
//            db.color.set(EColorData.POSITION_CONTROL_INPUT, (double)EColorData.EInput.POSITIVE.ordinal());
//            if (db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE).equals(EColorData.EMotorState.ON.ordinal()) ) {
//                mVictor.set(ControlMode.PercentOutput, Settings.kDJBoothOuput );
//            }
//            else {
//                mVictor.set(ControlMode.PercentOutput, 0d );
//            }
//        }
//        else if (db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_ROTATION_CONTROL) ) {
//            db.color.set(EColorData.ROTATION_CONTROL_INPUT, (double)EColorData.EInput.POSITIVE.ordinal());
//            if (db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE).equals(EColorData.EMotorState.ON.ordinal()) ) {
//                mVictor.set(ControlMode.PercentOutput, Settings.kDJBoothOuput );
//            }
//            else {
//                mVictor.set(ControlMode.PercentOutput, 0d );
//            }
//        }
//        else {
//            db.color.set(EColorData.POSITION_CONTROL_INPUT, (double)EColorData.EInput.NEGATIVE.ordinal());
//            db.color.set(EColorData.ROTATION_CONTROL_INPUT, (double)EColorData.EInput.NEGATIVE.ordinal());
//        }

//    }
//
//    public void updateArm(double pNow) {
//        if (db.operatorinput.isSet(InputMap.OPERATOR.HIGHER_ARM)) {
//            mArmState = PowerCellModule.EArmState.ENGAGED;
//        } else {
//            mArmState = PowerCellModule.EArmState.DISENGAGED;
//        }
//        switch (mArmState) {
//                db.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 1.0);
//                break;
//            case DISENGAGED:
//                db.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 0.0);
//                break;
//        }
//        TODO default state
//    }

