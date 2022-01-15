package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EColorData;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.Field2020;
import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;

import us.ilite.common.Data;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.*;
import static us.ilite.common.types.EVisionGoal2020.*;

import us.ilite.robot.modules.DJSpinnerModule;
import us.ilite.robot.modules.Limelight;
import static us.ilite.robot.Enums.*;
import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;
import static us.ilite.robot.modules.DriveModule.kDriveNEOVelocityFactor;

public class TestController extends BaseManualController {

    private ILog mLog = Logger.createLog(TestController.class);

    private Double mLastTargetTrackingType = 0d;
    private Double mLastGroundTrackingType = 0d;

    private double mLimelightZoomThreshold = 7.0;
    private double mLimelightGoalThreshold = 5.0;
    private Double mLastTrackingType = 0d;
    private Joystick mFlywheelJoystick;
    private Joystick mLimelightJoystick;
    public final RobotCodex<ELogitech310> flywheelinput = new RobotCodex(Data.NULL_CODEX_VALUE, ELogitech310.class);
    public final RobotCodex<ELogitech310> limelightinput = new RobotCodex(Data.NULL_CODEX_VALUE, ELogitech310.class);

    private double mStartTime;

    private EIntakeState mIntakeState;
    private EArmState mArmState;
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
        mFlywheelJoystick = new Joystick(2);
        mLimelightJoystick = new Joystick(3);
        db.registerAllWithShuffleboard();
    }

    private double mMaxSpeed = 0.0;
    private double mMaxYaw = 0.0;

    @Override
    protected void updateImpl() {
        ELogitech310.map(flywheelinput, mFlywheelJoystick);
        ELogitech310.map(limelightinput, mLimelightJoystick);

        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        clock.report("updateLimelightTargetLock", t -> updateTargetTracking());
        clock.report("updateFlywheel", t -> updateFlywheel());
        clock.report("updateDrivetrain", t -> updateDrivetrain());
        clock.report("updateIntake", t -> updatePowerCells());
        clock.report("updateDJBooth", t -> updateDJBooth());
//        updateArm(pNow);

        double spd = Math.max(db.drivetrain.get(R_ACTUAL_VEL_FT_s), db.drivetrain.get(L_ACTUAL_VEL_FT_s));
        mMaxSpeed = Math.max(mMaxSpeed, spd);
        SmartDashboard.putNumber("Max Robot Speed (ft/s)", mMaxSpeed);
        SmartDashboard.putNumber("Max Drive RPM", mMaxSpeed / kDriveNEOVelocityFactor);

        mMaxYaw = Math.max(mMaxYaw, db.imu.get(EGyro.YAW_OMEGA_DEGREES));
        SmartDashboard.putNumber("Max Robot Omega (deg/s)", mMaxYaw);
    }

    private void updateFlywheel() {
//        db.flywheel.set(TURRET_CONTROL, TurretControlType.MANUAL);
//        double turretDirection = db.operatorinput.get(InputMap.OPERATOR_REFACTOR.MANUAL_TURRET);
//        turretDirection = Math.abs(turretDirection) > 0.02 ? turretDirection : 0.0; //Handling Deadband
//        db.flywheel.set(MANUAL_TURRET_DIRECTION, turretDirection);

        if (db.groundTracking.isSet(TX)) {
            db.flywheel.set(SET_TURRET_ANGLE_deg, db.goaltracking.get(TARGET_AZIMUTH_deg));
        } else {
            db.flywheel.set(SET_TURRET_ANGLE_deg, 0);
        }
        if (flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_10_TEST)) {
            firingSequence(FlywheelSpeeds.CLOSE);
        } else if (flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_20_TEST)) {
            firingSequence(FlywheelSpeeds.INITIATION_LINE);
        } else if (flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_30_TEST)) {
            firingSequence(FlywheelSpeeds.FAR);
        } else if (flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_40_TEST)) {
            firingSequence(FlywheelSpeeds.FAR_TRENCH);
        } else {
            firingSequence(FlywheelSpeeds.OFF);
        }

        if(db.flywheel.isSet(HOOD_SENSOR_ERROR)) {
            db.flywheel.set(HOOD_STATE, Enums.HoodState.NONE);
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.HOOD)) {
            db.flywheel.set(HOOD_STATE, Enums.HoodState.MANUAL);
            db.flywheel.set(SET_HOOD_pct, flywheelinput.get(InputMap.FLYWHEEL.HOOD));
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.HOOD_TO_ANGLE)){
            db.flywheel.set(HOOD_STATE, Enums.HoodState.TARGET_ANGLE);
            db.flywheel.set(SET_HOOD_ANGLE_deg, 45.0);
        } else {
            db.flywheel.set(HOOD_STATE, Enums.HoodState.MANUAL);
            db.flywheel.set(SET_HOOD_pct, 0.0);
        }

        Enums.FlywheelSpeeds state = Enums.FlywheelSpeeds.OFF;
        if(flywheelinput.isSet(InputMap.FLYWHEEL.FEEDER_SPINUP_TEST)) {
            db.flywheel.set(SET_FEEDER_pct, 0.75);
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_SPINUP_TEST)) {
            db.flywheel.set(FLYWHEEL_WHEEL_STATE, Enums.FlywheelWheelState.OPEN_LOOP);
            db.flywheel.set(FLYWHEEL_OPEN_LOOP, 0.2);
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_10_TEST)) {
            state = Enums.FlywheelSpeeds.CLOSE;
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_20_TEST)) {
            state = Enums.FlywheelSpeeds.INITIATION_LINE;
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_30_TEST)) {
            state = Enums.FlywheelSpeeds.FAR;
        } else if(flywheelinput.isSet(InputMap.FLYWHEEL.FLYWHEEL_VELOCITY_40_TEST)) {
            state = Enums.FlywheelSpeeds.FAR_TRENCH;
        } else {
            state = Enums.FlywheelSpeeds.OFF;
        }
        db.flywheel.set(FLYWHEEL_SPEED_STATE, state);
        setFlywheelClosedLoop(state, true);
        if(flywheelinput.isSet(InputMap.FLYWHEEL.TEST_FIRE) && isFlywheelUpToSpeed()) {
            db.flywheel.set(SET_FEEDER_pct, state.feeder);
            db.flywheel.set(SET_FEEDER_rpm, state.feeder * 11000.0);
        } else {
            db.flywheel.set(SET_FEEDER_pct, 0.0);
        }
//        if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_FLYWHEEL)) {
//            if (db.limelight.isSet(ELimelightData.TV)) {
//                SmartDashboard.putNumber("Distance To Target", db.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
//                if (db.limelight.get(ELimelightData.CALC_DIST_TO_TARGET) <= 50) {
//                    db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 100);
//                } else {
//                    db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 2000);
//                }
//            } else {
//                db.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 1000);
//            }
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
    }

    public void updateTargetTracking() {
        boolean isOffset = !(Robot.DATA.goaltracking.get(TS) > 0 - mLimelightGoalThreshold || Robot.DATA.goaltracking.get(TS) < -90 + mLimelightGoalThreshold);

        if (limelightinput.isSet(InputMap.LIMELIGHT.LIMELIGHT_LOCK_TARGET)) {
//            Robot.DATA.goaltracking.set(TARGET_ID, (Field2020.FieldElement.TARGET.id()));
        } else if (limelightinput.isSet(InputMap.LIMELIGHT.LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.goaltracking.isSet(TY)) {
                if (Math.abs(Robot.DATA.goaltracking.get(TX)) < mLimelightZoomThreshold) {
                    Robot.DATA.goaltracking.set(TARGET_ID, Field2020.FieldElement.TARGET_ZOOM.id());
                }
            } else {
//                Robot.DATA.goaltracking.set(TARGET_ID, Field2020.FieldElement.TARGET.id());
            }
        } else {
            Robot.DATA.goaltracking.set(TARGET_ID, Limelight.NONE.id());
        }
        if ((Robot.DATA.goaltracking.get(TARGET_ID.ordinal()) != (mLastTargetTrackingType))
                && !(Robot.DATA.goaltracking.get(TARGET_ID.ordinal()) == Limelight.NONE.id())) {
            //TODO TargetLock(); something to do with targetlock here, need clarification on command structure
        }
        mLastTargetTrackingType = Robot.DATA.goaltracking.get(TARGET_ID.ordinal());
    }

    public void updateGroundTracking() {
        if (limelightinput.isSet(InputMap.LIMELIGHT.LIMELIGHT_LOCK_BALL)) {
            Robot.DATA.groundTracking.set(TARGET_ID, Field2020.FieldElement.BALL.id());
        } else if (limelightinput.isSet(InputMap.LIMELIGHT.LIMELIGHT_LOCK_BALL_DUAL)) {
            Robot.DATA.groundTracking.set(TARGET_ID, Field2020.FieldElement.BALL_DUAL.id());
        } else if (limelightinput.isSet(InputMap.LIMELIGHT.LIMELIGHT_LOCK_BALL_TRI)) {
            Robot.DATA.groundTracking.set(TARGET_ID, Field2020.FieldElement.BALL_TRI.id());
        } else {
            Robot.DATA.groundTracking.set(TARGET_ID, RawLimelight.NONE.id());
        }
        if ((Robot.DATA.groundTracking.get(TARGET_ID.ordinal()) != (mLastGroundTrackingType))
                && !(Robot.DATA.groundTracking.get(TARGET_ID.ordinal()) == RawLimelight.NONE.id())) {
            //TODO TargetLock(); something to do with targetlock here, need clarification on command structure
        }
        mLastGroundTrackingType = Robot.DATA.goaltracking.get(TARGET_ID.ordinal());
    }

    protected void updatePowerCells() {
        if(db.driverinput.isSet(ELogitech310.A_BTN)) {
            db.intake.set(SET_INTAKE_VEL_ft_s, 0.2);
        }  else {
            db.intake.set(SET_INTAKE_VEL_ft_s, 0);
        }

        db.intake.set(SET_H_pct, db.driverinput.get(ELogitech310.RIGHT_Y_AXIS) / 3);
        SmartDashboard.putNumber("Right Axis Y Velocity", db.driverinput.get(ELogitech310.RIGHT_Y_AXIS));

        if(db.driverinput.isSet(ELogitech310.L_BTN)) {
            db.intake.set(LEFT_PNEUMATIC_STATE, 1.0);
            db.intake.set(RIGHT_PNEUMATIC_STATE, 1.0);
        }
        else if (db.driverinput.isSet(ELogitech310.R_BTN)) {
            db.intake.set(LEFT_PNEUMATIC_STATE, 0.0);
            db.intake.set(RIGHT_PNEUMATIC_STATE, 0.0);
        }
    }

    void updateDJBooth() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.COLOR_POSITION)) {
            int i = (int) db.color.get(EColorData.SENSED_COLOR);
            EColorMatch m = EColorMatch.values()[i];
            Color DJ_COLOR = null;
            switch (db.recieveColorFmsRelay()) {
                case 'B':
                    DJ_COLOR = EColorMatch.BLUE.color;
                    break;
                case 'G':
                    DJ_COLOR = EColorMatch.GREEN.color;
                    break;
                case 'R':
                    DJ_COLOR = EColorMatch.RED.color;
                    break;
                case 'Y':
                    DJ_COLOR = EColorMatch.YELLOW.color;
                    break;
                default:
                    DJ_COLOR = null;
                    break;
            }
            if ( DJ_COLOR == null ) {
                DriverStation.reportError("NO FMS RELAY RECEIVED! SWITCHING TO MANUAL!", false );
                db.color.set(EColorData.DESIRED_MOTOR_POWER, EColorWheelState.POSITION.power);
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) EColorWheelState.POSITION.ordinal());
            } else if (m.color.equals(DJ_COLOR)) {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, EColorWheelState.OFF.power);
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) EColorWheelState.POSITION.ordinal());
            } else {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, EColorWheelState.POSITION.getPower());
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, EColorWheelState.POSITION.ordinal());
            }
        }
        else if ( db.operatorinput.isSet(InputMap.OPERATOR.COLOR_ROTATION )) {
            if(db.color.get(EColorData.WHEEL_ROTATION_COUNT) >= DJSpinnerModule.TARGET_ROTATION_COUNT) {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, EColorWheelState.OFF.getPower());
            } else {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, EColorWheelState.ROTATION.getPower());
            }
            db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, EColorWheelState.ROTATION.ordinal());
        }
    }
}


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

