package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Field2020;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.*;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EPowerCellData;

import static us.ilite.common.config.InputMap.DRIVER.*;
import static us.ilite.common.types.drive.EDriveData.*;

public class TestController extends AbstractController {

    private ILog mLog = Logger.createLog(TestController.class);
    private Double mLastTrackingType;
    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private double mLimelightZoomThreshold = 7.0;

    private HangerModule.EHangerState mHangerState = HangerModule.EHangerState.NOT_HANGING;
    private PowerCellModule.EIntakeState mIntakeState = PowerCellModule.EIntakeState.INTAKE;
    private PowerCellModule.EArmState mArmState = PowerCellModule.EArmState.ENGAGED;
    private PowerCellModule.EIndexingState mIndexingState;
    private double mPreviousTime;

    public TestController() {
    }

    public void update(double pNow) {
        updateLimelightTargetLock();
        updateDrivetrain(pNow);
        updateFlywheel(pNow);
        updateIntake(pNow);
        updateHanger(pNow);
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
//        mPreviousTime = pNow;
//        mLog.error("-------------------------------------------------------Flywheel Velocity: ", db.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
    }

    public void updateLimelightTargetLock() {
        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.TY) != null) {
                SmartDashboard.putNumber("Distance to Target", Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
            }
            Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id() );
        } else if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET_ZOOM)) {
            if (Robot.DATA.selectedTarget.get(ELimelightData.TY) != null) {
                if (Math.abs(Robot.DATA.selectedTarget.get(ELimelightData.TX)) < mLimelightZoomThreshold) {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET_ZOOM.id());
                    System.out.println("ZOOMING");
                } else {
                    Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double) Field2020.FieldElement.TARGET.id() );
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
        }
        else {
                Robot.DATA.limelight.set(ELimelightData.TARGET_ID, (double)Limelight.NONE.id());
//            if(mTeleopCommandManager.isRunningCommands()) mTeleopCommandManager.stopRunningCommands(pNow);
        }
        if (!(Robot.DATA.limelight.get(ELimelightData.TARGET_ID.ordinal()).equals(mLastTrackingType) )
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
            var d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
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
            mIntakeState = PowerCellModule.EIntakeState.INTAKE;
            mArmState = PowerCellModule.EArmState.ENGAGED;

            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle());

            if(db.powercell.get(EPowerCellData.CURRENT_INDEXING_STATE) == (double) PowerCellModule.EIndexingState.INDEXING.ordinal()) {
                db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, mIntakeState.getPower());
                db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, mIntakeState.getPower());
            }
            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, db.drivetrain.get(LEFT_VEL_IPS) + PowerCellModule.kDeltaIntakeVel);

        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
            mIntakeState = PowerCellModule.EIntakeState.REVERSE;
            mArmState = PowerCellModule.EArmState.ENGAGED;

            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle());

            if(db.powercell.get(EPowerCellData.CURRENT_INDEXING_STATE) == (double) PowerCellModule.EIndexingState.INDEXING.ordinal()) {
                db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, mIntakeState.getPower());
                db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, mIntakeState.getPower());
            }

            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, db.drivetrain.get(LEFT_VEL_IPS) + PowerCellModule.kDeltaIntakeVel);

        } else {
            mIntakeState = PowerCellModule.EIntakeState.STOP;
            mArmState = PowerCellModule.EArmState.DISENGAGED;

            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, mArmState.getAngle());

            if(db.powercell.get(EPowerCellData.CURRENT_INDEXING_STATE) == (double) PowerCellModule.EIndexingState.NOT_INDEXING.ordinal()) {
                db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY, mIntakeState.getPower());
                db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY, mIntakeState.getPower());
            }

            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, db.drivetrain.get(LEFT_VEL_IPS) + PowerCellModule.kDeltaIntakeVel);
        }

        //Testing if statement

//        if(db.operatorinput.isSet(InputMap.OPERATOR.INTAKE)) {
//            mLog.error("-------------------------------------------------------ispressed---------------");
//            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, 0.25);
//        } else {
//            db.powercell.set(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S, 0.0);
//        }


    }

//    Arm is in intake update
//    public void updateArm(double pNow) {
//        if(db.operatorinput.isSet(InputMap.OPERATOR.HIGHER_ARM)) {
//            mLog.error("-------------------------------------------------------ispressed---------------");
//            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, 0.25);
//        } else {
//            db.powercell.set(EPowerCellData.DESIRED_ARM_ANGLE, 0.0);
//        }
//    }
}