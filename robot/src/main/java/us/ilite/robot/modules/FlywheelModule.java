package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {
    public CANSparkMax mShooter;
    private Servo mHoodAngler;
    private TalonSRX mTurret;
    private TalonSRX mAccelerator;
    private ETrackingType mTrackingType = ETrackingType.NONE;
    private ETurretMode mTurretMode = ETurretMode.GYRO;

    public FlywheelModule() {
        mShooter = SparkMaxFactory.createDefaultSparkMax(Settings.ShooterSystem.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAccelerator = new TalonSRX(Settings.ShooterSystem.kAcceleratorID);
        mHoodAngler = new Servo(Settings.ShooterSystem.kAnglerID);
        mTurret = new TalonSRX(Settings.ShooterSystem.kTurretID);
    }

    public enum EShooterState {
        SHOOT,
        STOP
    }

    public enum ETurretMode {
        GYRO,
        LIMELIGHT
    }

    public enum EAcceleratorState {
        FEED,
        STOP
    }

    public enum EHoodState {
        STATIONARY,
        ADJUSTABLE
    }

    public double calcSpeedFromDistance(double distance) { return 0.09 * Math.pow(distance, 2) + 8.0; } // Need recalculation
    public double calcAngleFromDistance(double distance, double height) { return Math.atan(height / distance); }

    public boolean isMaxVelocity() { return mShooter.getEncoder().getVelocity() >= Settings.ShooterSystem.kAcceleratorThreshold; }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0.0);
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mShooter.getEncoder().getVelocity());

        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_LIMELIGHT_TARGET, (double) mTrackingType.ordinal());
        if (Robot.DATA.limelight.isSet(ETargetingData.targetOrdinal)) {
            Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_LIMELIGHT_TARGET, Robot.DATA.limelight.get(ETargetingData.targetOrdinal));
        }

        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0.0);
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY, (double) mTurret.getSelectedSensorVelocity());

        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_HOOD_ANGLE, Settings.ShooterSystem.kBaseHoodAngle);
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_HOOD_ANGLE, mHoodAngler.getAngle());

        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_ACCELERATOR_VELOCITY, 0.0);
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, (double)mAccelerator.getSelectedSensorVelocity());

        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_MODE, (double) mTurretMode.ordinal());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_MODE, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_MODE));


    }

    @Override
    public void setOutputs(double pNow) {
        if ( isMaxVelocity() ) {
            mAccelerator.set(ControlMode.PercentOutput, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_ACCELERATOR_VELOCITY));
        }
        else {
            mAccelerator.set(ControlMode.PercentOutput, 0.0);
        }
        mHoodAngler.setAngle(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_HOOD_ANGLE));
        mShooter.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mTurret.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_VELOCITY));
    }

    @Override
    public void shutdown(double pNow) {

    }
}