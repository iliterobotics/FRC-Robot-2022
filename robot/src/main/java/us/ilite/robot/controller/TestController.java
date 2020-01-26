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

    private boolean isGyro;

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
        if (isGyro) {
            robotHeading = mTurretGyro.getCompassHeading();
        } else if (mData.limelight.get(ETargetingData.tx) != null ) {
            robotHeading = mData.limelight.get(ETargetingData.tx);
        }

        if (mData.driverinput.isSet(InputMap.DRIVER.SHOOT)) {
            Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-10 * robotHeading, pTime - mPreviousTime));
            Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, mShooter.targetValid() ? mShooter.calcAngleFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget), mData.limelight.get(ETargetingData.ty)) : 60);

            isGyro = false;
        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-2 * robotHeading, pTime - mPreviousTime));
            isGyro = true;
        }

        mPreviousTime = pTime;
    }

    public void update(double pNow) {
        updateShooter(pNow);
    }
}