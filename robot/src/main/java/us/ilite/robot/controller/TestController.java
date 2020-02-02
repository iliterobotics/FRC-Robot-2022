package us.ilite.robot.controller;

import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.input.EInputScale;
import us.ilite.common.types.EColorData;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Field2020;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.*;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;

import static us.ilite.common.config.InputMap.DRIVER.*;
import static us.ilite.common.types.drive.EDriveData.*;

public class TestController extends AbstractController {

    private ILog mLog = Logger.createLog(TestController.class);
    private Double mLastTrackingType;
    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    private double mLimelightZoomThreshold = 7.0;

    private HangerModule.EHangerState mHangerState;
    private PowerCellModule.EIntakeState mIntakeState;
    private PowerCellModule.EArmState mArmState;
    private double mPreviousTime;

    public TestController() {
    }

    public void update(double pNow) {
        updateLimelightTargetLock(pNow);
        updateDrivetrain(pNow);
        updateFlywheel(pNow);
        updateIntake(pNow);
        updateHanger(pNow);
        updateDJBooth();
//        updateArm(pNow);
    }

    private void updateHanger(double pNow){
        if (Robot.DATA.operatorinput.isSet(InputMap.DRIVER.BEGIN_HANG)){
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , 1.0);
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , 1.0);

        }
        else if (Robot.DATA.operatorinput.isSet(InputMap.DRIVER.RELEASE_HANG)){
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , 0.0);
            Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , 0.0);

        }
        switch (mHangerState){
            case HANGING:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , 1.0);
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , 1.0);
            case NOT_HANGING:
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER1 , 0.0);
                Robot.DATA.hanger.set(EHangerModuleData.DESIRED_HANGER_POWER2 , 0.0);
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
        mPreviousTime = pNow;
        mLog.error("-------------------------------------------------------Flywheel Velocity: ", db.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
    }

    public void updateLimelightTargetLock(double pNow) {
        System.out.println(db.limelight);
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
//            if (Robot.DATA.groundTracking.isSet(ELimelightData.TY)) {
//                SmartDashboard.putNumber("Distance to Target", Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
//            }
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.TARGET.id() );
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.groundTracking.isSet(ELimelightData.TY)) {
                if (Math.abs(Robot.DATA.groundTracking.get(ELimelightData.TX)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.TARGET_ZOOM.id());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.TARGET.id() );
                }
            } else {
                   Robot.DATA.groundTracking.set(ELimelightData.TARGET_ID, Field2020.FieldElement.TARGET.id());
            }
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.BALL.id());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_DUAL)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.BALL_DUAL.id());
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_BALL_TRI)) {
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Field2020.FieldElement.BALL_TRI.id());
        }
        else {
                Robot.DATA.limelight.set(ELimelightData.TARGET_ID, Limelight.NONE.id());
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if ((Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) != (mLastTrackingType) )
                && !(Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()) == Limelight.NONE.id())) {
                mLog.error("Requesting command start");
                mLog.error("Stopping teleop command queue");
//            mTeleopCommandManager.stopRunningCommands(pNow);
//            mTeleopCommandManager.startCommands(new LimelightTargetLock(mDrive, mLimelight, 2, mTrackingType, this, false).setStopWhenTargetLost(false));
        }
        mLastTrackingType =  Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal());
    }

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate = Math.abs(rotate) > 0.01 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.01 ? throttle : 0.0; //Handling Deadband

        db.drivetrain.set(SHOULD_HOLD_POSITION, (throttle == 0.0 && rotate == 0.0) ? 1.0 : 0.0);

        if (throttle == 0.0 && rotate == 0.0) {
            db.drivetrain.set(DESIRED_THROTTLE, 0.0);
            db.drivetrain.set(DESIRED_TURN, 0.0);
        } else {
            db.drivetrain.set(SHOULD_HOLD_POSITION, 0.0);
            if (throttle == 0.0 && rotate != 0.0) {
                throttle += 0.03;
            }
            DriveMessage d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
            throttle = d.getThrottle();
            rotate = d.getTurn();
            if (db.driverinput.isSet(SUB_WARP_AXIS) && db.driverinput.get(SUB_WARP_AXIS) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
                throttle *= Settings.Input.kSnailModePercentThrottleReduction;
                rotate *= Settings.Input.kSnailModePercentRotateReduction;
            }
            db.drivetrain.set(DESIRED_THROTTLE, -throttle);
            db.drivetrain.set(DESIRED_TURN, rotate);
        }

    }

    private void updateIntake(double pNow) {
        if (db.operatorinput.isSet(InputMap.OPERATOR.INTAKE)) {
            mLog.error("--------------INTAKE IS BEING PRESSED----------");
            mIntakeState = PowerCellModule.EIntakeState.INTAKE;
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
            mIntakeState = PowerCellModule.EIntakeState.REVERSE;
        } else {
            mIntakeState = PowerCellModule.EIntakeState.STOP;
        }
        switch (mIntakeState) {
            case INTAKE:
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , 1.0);
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , 1.0);
                db.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , 1.0);
                break;
            case REVERSE:
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , -1.0);
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , -1.0);
                db.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , -1.0);
                break;
            case STOP:
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , 0.0);
                db.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , 0.0);
                db.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , 0.0);
                break;
        }
    }



    void updateDJBooth() {
        if ( db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL)) {
            DJSpinnerModule.EColorMatch m =db.color.get(EColorData.SENSED_COLOR, DJSpinnerModule.EColorMatch.class);
            if(m.color.equals(db.DJ_COLOR)) {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.OFF.power);
            } else {
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.POSITION.power);
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) DJSpinnerModule.EColorWheelState.POSITION.ordinal());
            }
        }

        if ( db.operatorinput.isSet(InputMap.OPERATOR.OPERATOR_ROTATION_CONTROL)) {

            if(db.color.get(EColorData.WHEEL_ROTATION_COUNT) >= DJSpinnerModule.sTARGET_ROTATION_COUNT) {
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) DJSpinnerModule.EColorWheelState.OFF.ordinal());
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.OFF.power);
            } else {
                db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, (double) DJSpinnerModule.EColorWheelState.ROTATION.ordinal());
                db.color.set(EColorData.DESIRED_MOTOR_POWER, DJSpinnerModule.EColorWheelState.ROTATION.power);
            }
        }



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

    }

    public void updateArm(double pNow) {
        if (db.operatorinput.isSet(InputMap.OPERATOR.HIGHER_ARM)) {
            mArmState = PowerCellModule.EArmState.ENGAGED;
        } else {
            mArmState = PowerCellModule.EArmState.DISENGAGED;
        }
        switch (mArmState) {
            case ENGAGED:
                db.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 1.0);
                break;
            case DISENGAGED:
                db.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 0.0);
                break;
        }
        //TODO default state
    }
}
