package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.FlywheelModule;

public class TestController extends AbstractController {
    private FlywheelModule mShooter;
    private Data mData;

    private PigeonIMU mTurretGyro;
    private PIDController mTurretPid;
    private PIDController mShooterPid;

    private FlywheelModule.EShooterState mShooterState;
    private FlywheelModule.EAcceleratorState mAcceleratorState;
    private FlywheelModule.ETurretMode mTurretMode;
    private FlywheelModule.EHoodState mHoodState;

    private boolean isGyro = true;

    private double mPreviousTime;
    private double robotHeading;

    public TestController(Data pData, FlywheelModule pShooter) {
        mData = pData;
        mShooter = pShooter;
        mTurretGyro = new PigeonIMU(Settings.ShooterSystem.kTurretGyroID);
        mTurretPid = new PIDController(Settings.ShooterSystem.kTurretAngleLockGains, -1, 1, Settings.kControlLoopPeriod);
        mShooterPid = new PIDController(Settings.ShooterSystem.kShooterGains, 0, 1, Settings.kControlLoopPeriod);
    }

    private void updateShooter(double pTime) {
        if (mData.driverinput.isSet(InputMap.DRIVER.SHOOT)) {
            Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-10 * robotHeading, pTime - mPreviousTime));
            Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Robot.DATA.limelight.isSet(ETargetingData.ty) ?
            mShooter.calcAngleFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget), mData.limelight.get(ETargetingData.ty)) : Settings.ShooterSystem.kBaseHoodAngle);
            mTurretMode = FlywheelModule.ETurretMode.LIMELIGHT;
            mShooterState = FlywheelModule.EShooterState.SHOOT;
        }
        else {
            mTurretMode = FlywheelModule.ETurretMode.GYRO;
            mShooterState = FlywheelModule.EShooterState.STOP;
        }


    }

    public void update(double pNow) {
        updateShooter(pNow);
        switch(mTurretMode) {
            case GYRO: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-2 * mTurretGyro.getCompassHeading(), pNow - mPreviousTime));
                break;
            case LIMELIGHT: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-10 * Robot.DATA.limelight.get(ETargetingData.tx), pNow - mPreviousTime));
                break;
        }
        switch(mShooterState) {
            case SHOOT:
                if ( Robot.DATA.limelight.isSet(ETargetingData.tx)) {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, mShooter.calcSpeedFromDistance(Robot.DATA.limelight.get(ETargetingData.calcDistToTarget)));
                }
                else {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, mShooterPid.calculate(Settings.ShooterSystem.kShooterTargetVelocity, pNow - mPreviousTime));
                }
        }
        switch(mHoodState) {
            case BASE: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
            break;
            case ADJUSTABLE:
                if (Robot.DATA.limelight.isSet(ETargetingData.ty)) {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, mShooter.calcAngleFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget), mData.limelight.get(ETargetingData.ty)));
                }
                else {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
                }
        }

        mPreviousTime = pNow;
    }
}