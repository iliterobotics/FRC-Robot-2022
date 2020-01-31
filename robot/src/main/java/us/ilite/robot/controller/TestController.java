package us.ilite.robot.controller;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.ETargetingData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.FlywheelModule;

public class TestController extends AbstractController {
    private FlywheelModule mShooter;
    private ILog mLog = Logger.createLog(this.getClass());

    private PigeonIMU mTurretGyro;
    private PIDController mTurretPid;
    private PIDController mShooterPid;

    private FlywheelModule.EShooterState mShooterState;
    private FlywheelModule.EAcceleratorState mAcceleratorState;
    private FlywheelModule.ETurretMode mTurretMode;
    private FlywheelModule.EHoodState mHoodState;

    private double mPreviousTime;

    public TestController(FlywheelModule pShooter) {
        mShooter = pShooter;
        mTurretGyro = new PigeonIMU(Settings.ShooterSystem.kTurretGyroID);
        mTurretPid = new PIDController(Settings.ShooterSystem.kTurretAngleLockGains, -1, 1, Settings.kControlLoopPeriod);
        mShooterPid = new PIDController(Settings.ShooterSystem.kShooterGains, 0, 1, Settings.kControlLoopPeriod);
    }

    public void update(double pNow) {
        if (!Robot.DATA.driverinput.get(InputMap.OPERATOR.SHOOT).equals(0.0) ) {
            mTurretMode = FlywheelModule.ETurretMode.LIMELIGHT;
            mHoodState = FlywheelModule.EHoodState.ADJUSTABLE;
            mAcceleratorState = FlywheelModule.EAcceleratorState.FEED;
            mShooterState = FlywheelModule.EShooterState.SHOOT;
        }
        else {
            mTurretMode = FlywheelModule.ETurretMode.GYRO;
            mAcceleratorState = FlywheelModule.EAcceleratorState.STOP;
            mShooterState = FlywheelModule.EShooterState.STOP;
            mHoodState = FlywheelModule.EHoodState.STATIONARY;
        }

        switch(mAcceleratorState) {
            case FEED: Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, Settings.ShooterSystem.kAcceleratorTargetVelocity);
                break;
            case STOP: Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, 0.0);
                break;
        }

        switch(mTurretMode) {
            case GYRO: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-2 * mTurretGyro.getCompassHeading(), pNow - mPreviousTime));
                break;
            case LIMELIGHT:
                if ( Robot.DATA.limelight.isSet(ETargetingData.tx)) {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, mTurretPid.calculate(-10 * Robot.DATA.limelight.get(ETargetingData.tx), pNow - mPreviousTime));
                }
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
                break;
            case STOP: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0.0);
                break;
        }

        switch(mHoodState) {
            case STATIONARY: Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
                break;
            case ADJUSTABLE:
                if (Robot.DATA.limelight.isSet(ETargetingData.ty)) {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, mShooter.calcAngleFromDistance(Robot.DATA.limelight.get(ETargetingData.calcDistToTarget), Robot.DATA.limelight.get(ETargetingData.ty)));
                }
                else {
                    Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
                }
        }
        mPreviousTime = pNow;

        SmartDashboard.putNumber("Flywheel Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Turret Mode", Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_MODE));
        SmartDashboard.putNumber("Accelerator Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY));
        SmartDashboard.putNumber("Hood Angle", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_HOOD_ANGLE));
    }
}