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
import us.ilite.common.types.EShooterSystemData;
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
        if(db.driverinput.isSet(ELogitech310.B_BTN))
        {
            db.flywheel.set(SET_BALL_VELOCITY_ft_s,0.1);
        }

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
        if(flywheelinput.isSet(InputMap.FLYWHEEL.RESET_INTAKE_COUNT)) {
            resetSerializerState();
        }
        // Default to none
        db.powercell.set(INTAKE_STATE, EArmState.NONE);

        if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_ACTIVATE) || flywheelinput.isSet(InputMap.FLYWHEEL.BASIC_INTAKE)) {
            setIntakeArmEnabled(true);
            activateSerializer();
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_REVERSE) || flywheelinput.isSet(InputMap.FLYWHEEL.REVERSE_INTAKE)) {
            db.powercell.set(INTAKE_STATE, EArmState.STOW);
            reverseSerializer();
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE_STOW) || flywheelinput.isSet(InputMap.FLYWHEEL.INTAKE_STOW)) {
            setIntakeArmEnabled(false);
            activateSerializer();
        } else {
            // TODO - only enable once we have set the hold gains
//            db.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.HOLD);
            db.powercell.set(INTAKE_STATE, EArmState.NONE);
            db.powercell.set(SET_INTAKE_VEL_ft_s, 0d);
        }

        if((db.driverinput.isSet(InputMap.DRIVER.FIRE_POWER_CELLS) || flywheelinput.isSet(InputMap.FLYWHEEL.TEST_FIRE)) && isFlywheelUpToSpeed() && isFeederUpToSpeed()) {
            db.powercell.set(SET_V_pct, 0.6);
            db.powercell.set(SET_H_pct, 0.5);
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

