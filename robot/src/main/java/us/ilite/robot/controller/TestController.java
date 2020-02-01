package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveMessage;
import us.ilite.robot.modules.PowerCellModule;

import static us.ilite.common.config.InputMap.DRIVER.*;
import static us.ilite.common.types.drive.EDriveData.THROTTLE;
import static us.ilite.common.types.drive.EDriveData.TURN;

public class TestController extends AbstractController {
    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(this.getClass());


    private PowerCellModule.EIntakeState mIntakeState;
    private PowerCellModule.EArmState mArmState;

    private double mPreviousTime;

    public TestController() {
    }

    public void update(double pNow) {
        updateDrivetrain(pNow);
        updateFlywheel(pNow);
        updateIntake(pNow);
//        updateArm(pNow);
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

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
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
        db.drivetrain.set(THROTTLE, throttle);
        db.drivetrain.set(TURN, rotate);
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