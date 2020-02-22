package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.Data;
import us.ilite.common.Field2020;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EColorData;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DJSpinnerModule;
import us.ilite.robot.modules.Limelight;

import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.EPowerCellData.DESIRED_H_VELOCITY;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;
import static us.ilite.robot.Robot.DATA;
import static us.ilite.robot.modules.DriveModule.kDriveNEOVelocityFactor;

public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private Double mLastTrackingType = 0d;
    private Joystick mFlywheelJoystick;
    public final RobotCodex<ELogitech310> flywheelinput = new RobotCodex(Data.NULL_CODEX_VALUE, ELogitech310.class);

    private double mLimelightZoomThreshold = 7.0;
    private double mStartTime;

    private Enums.EIntakeState mIntakeState;
    private Enums.EArmState mArmState;
    private double mPreviousTime;
    private double mGoalBeamCountBroken = 0;
    private boolean crossedEntry = false;
    private Boolean allBeamsBroken = false;

    private static TeleopController INSTANCE;

    public static TeleopController getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
        mFlywheelJoystick = new Joystick(2);
        //db.registerAllWithShuffleboard();
    }

    private double mMaxSpeed = 0.0;
    private double mMaxYaw = 0.0;
    @Override
    protected void updateImpl(double pNow) {
        //super.updateDrivetrain(pNow);

        ELogitech310.map(flywheelinput, mFlywheelJoystick);

        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        //updateLimelightTargetLock();
        //updateFlywheel(pNow);
        updateDrivetrain(pNow);
        //updatePowerCells(pNow);
        //updateHanger(pNow);
        //updateDJBooth(pNow);
        //updateArm(pNow);

        double spd = Math.max(db.drivetrain.get(R_ACTUAL_VEL_FT_s), db.drivetrain.get(L_ACTUAL_VEL_FT_s));
        mMaxSpeed = Math.max(mMaxSpeed, spd);
        SmartDashboard.putNumber("Max Robot Speed (ft/s)", mMaxSpeed);
        SmartDashboard.putNumber("Max Drive RPM", mMaxSpeed / kDriveNEOVelocityFactor);

        mMaxYaw = Math.max(mMaxYaw, db.imu.get(EGyro.YAW_OMEGA_DEGREES));
        SmartDashboard.putNumber("Max Robot Omega (deg/s)", mMaxYaw);
    }

//    private void updateHanger(double pNow) {
//        if (db.operatorinput.isSet(InputMap.OPERATOR.BEGIN_HANG)) {
//            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_POSITION, 17.0);
//        } else {
//            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_POSITION, 0.0);
//        }
//    }
//
//    private void updateFlywheel(double pNow) {
//        if(db.flywheel.isSet(HOOD_SENSOR_ERROR)) {
//            db.flywheel.set(HOOD_STATE, Enums.HoodState.NONE);
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.HOOD)) {
//            db.flywheel.set(HOOD_STATE, Enums.HoodState.MANUAL);
//            db.flywheel.set(HOOD_OPEN_LOOP, flywheelinput.get(InputMap.FLYWHEEL.HOOD));
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.HOOD_TO_ANGLE)){
//            db.flywheel.set(HOOD_STATE, Enums.HoodState.TARGET_ANGLE);
//            db.flywheel.set(TARGET_HOOD_ANGLE, 45.0);
//        } else {
//            db.flywheel.set(HOOD_STATE, Enums.HoodState.MANUAL);
//            db.flywheel.set(HOOD_OPEN_LOOP, 0.0);
//        }
//
//
//        Enums.FlywheelSpeeds state = Enums.FlywheelSpeeds.OFF;
//        if(flywheelinput.isSet(InputMap.FLYWHEEL.FEEDER_SPINUP_TEST)) {
//            db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP, 0.75);
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_SPINUP_TEST)) {
//            db.flywheel.set(FLYWHEEL_WHEEL_STATE, Enums.FlywheelWheelState.OPEN_LOOP);
//            db.flywheel.set(FLYWHEEL_OPEN_LOOP, 0.2);
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_10_TEST)) {
//            state = Enums.FlywheelSpeeds.CLOSE;
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_20_TEST)) {
//            state = Enums.FlywheelSpeeds.INITIATION_LINE;
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_30_TEST)) {
//            state = Enums.FlywheelSpeeds.FAR;
//        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_40_TEST)) {
//            state = Enums.FlywheelSpeeds.FAR_TRENCH;
//        } else {
//            state = Enums.FlywheelSpeeds.OFF;
//        }
//        db.flywheel.set(FLYWHEEL_SPEED_STATE, state);
//        setFlywheelClosedLoop(state);
//        if(flywheelinput.isSet(InputMap.FLYWHEEL.TEST_FIRE) && isFlywheelUpToSpeed()) {
//            db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP, state.feeder);
//            db.flywheel.set(TARGET_FEEDER_VELOCITY_RPM, state.feeder * 11000.0);
//        } else {
//            db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP, 0.0);
//        }
//    }
//
//    public void updateLimelightTargetLock() {
//        if (DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
//            if (DATA.selectedTarget.isSet(ELimelightData.TY)) {
//                SmartDashboard.putNumber("Distance to Target", DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
//            }
//            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
//            if (Robot.DATA.selectedTarget.isSet(ELimelightData.TY)) {
//                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.TX)) < mLimelightZoomThreshold) {
//                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET_ZOOM.id());
//                    System.out.println("ZOOMING");
//                } else {
//                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//                }
//            } else {
//                Robot.DATA.selectedTarget.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id());
//            }
//        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
//            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL.id());
//        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
//            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_DUAL.id());
//            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.BALL_TRI.id());
//        } else {
//            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Limelight.NONE.id());
////            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
//        }
//        if ((Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) != (mLastTrackingType))
//                && !(Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) == Limelight.NONE.id())) {
//            mLog.error("Requesting command start");
//            mLog.error("Stopping teleop command queue");
////            mTeleopCommandManager.stopRunningCommands(pNow);
////            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
//        }
//        mLastTrackingType = Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal());
//    }
//
//    protected void updatePowerCells(double pNow) {
//        // Default to none
//        db.powercell.set(INTAKE_STATE, Enums.EArmState.NONE);
//
//        if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_ACTIVATE) || flywheelinput.isSet(InputMap.FLYWHEEL.BASIC_INTAKE)) {
//            setIntakeArmEnabled(pNow, true);
//            crossedEntry = activateSerializer(pNow);
////            if (crossedEntry && !db.powercell.isSet(EPowerCellData.SECONDARY_BREAM_BREAKER)) {
////                db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, power);
////                db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, power);
////            } else if ( db.powercell.isSet(EPowerCellData.SECONDARY_BREAM_BREAKER) ) {
////                crossedEntry = false;
////            }
//
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_REVERSE) || flywheelinput.isSet(InputMap.FLYWHEEL.REVERSE_INTAKE)) {
//            db.powercell.set(INTAKE_STATE, Enums.EArmState.STOW);
//            reverseSerializer(pNow);
//        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_STOW) || flywheelinput.isSet(InputMap.FLYWHEEL.INTAKE_STOW)) {
//            setIntakeArmEnabled(pNow, false);
//            crossedEntry = activateSerializer(pNow);
//        } else {
//            // TODO - only enable once we have set the hold gains
////            db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.HOLD);
//            db.powercell.set(INTAKE_STATE, Enums.EArmState.NONE);
//            db.powercell.set(DESIRED_INTAKE_VELOCITY_FT_S, 0d);
//        }
//
//        if ((db.driverinput.isSet(InputMap.DRIVER.FIRE_POWER_CELLS) || flywheelinput.isSet(InputMap.FLYWHEEL.TEST_FIRE)) && isFlywheelUpToSpeed() && isFeederUpToSpeed()) {
//            db.powercell.set(DESIRED_V_VELOCITY, 0.6);
//            db.powercell.set(DESIRED_H_VELOCITY, 0.5);
//        }
//    }
//
//    void updateDJBooth(double pNow) {
//        if (db.operatorinput.isSet(InputMap.OPERATOR.COLOR_POSITION)) {
//            db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.POSITION.getPower());
//            int i = (int) db.color.get(EColorData.SENSED_COLOR);
//            Enums.EColorMatch m = Enums.EColorMatch.values()[i];
//            Color DJ_COLOR = null;
//            switch (db.recieveColorFmsRelay()) {
//                case 'B':
//                    DJ_COLOR = Enums.EColorMatch.BLUE.color;
//                    break;
//                case 'G':
//                    DJ_COLOR = Enums.EColorMatch.GREEN.color;
//                    break;
//                case 'R':
//                    DJ_COLOR = Enums.EColorMatch.RED.color;
//                    break;
//                case 'Y':
//                    DJ_COLOR = Enums.EColorMatch.YELLOW.color;
//                    break;
//                default:
//                    DJ_COLOR = null;
//                    break;
//            }
//            if (m.color.equals(DJ_COLOR)) {
//                //TODO stop using the module for the desired power
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.OFF.getPower());
//            } else {
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.POSITION.getPower());
//                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, Enums.EColorWheelState.POSITION.ordinal());
//            }
//        }
//        else if ( db.operatorinput.isSet(InputMap.OPERATOR.COLOR_ROTATION)) {
//            db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.ROTATION.getPower());
//            if(db.color.get(EColorData.WHEEL_ROTATION_COUNT) >= DJSpinnerModule.TARGET_ROTATION_COUNT) {
//                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, Enums.EColorWheelState.OFF.ordinal());
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.OFF.getPower());
//            } else {
//                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, Enums.EColorWheelState.ROTATION.ordinal());
//                db.color.set(EColorData.DESIRED_MOTOR_POWER, Enums.EColorWheelState.ROTATION.getPower());
//            }
//        }
//
//    }

}
