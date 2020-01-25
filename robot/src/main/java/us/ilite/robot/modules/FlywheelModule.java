package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.EShooterData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelModule extends Module {

    private Data mData;
    private Limelight mLimelight;
    private PigeonIMU mTurretGyro;
    private PIDController mShooterPid;

    private CANSparkMax mShooter;
    private Servo mHoodAngler;
    private TalonSRX mTurret;
    private CANSparkMax mAccelerator;

    private EShooterState mShooterState = EShooterState.STOP;
    private EAcceleratorState mAcceleratorState = EAcceleratorState.STOP;
    private ETrackingType mTrackingType = ETrackingType.NONE;

    private double mPreviousTime= 0;
    private double desiredHoodAngle = 60;
    private double desiredShooterVelocity = 0;
    private double desiredTurretVelocity = 0;
    private double desiredAcceleratorVelocity = 0;
    private double robotHeading;

    public boolean isGyro;

    public FlywheelModule(Data pData, Limelight pLimelight) {
        mShooter = SparkMaxFactory.createDefaultSparkMax(Settings.ShooterSystem.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAccelerator = SparkMaxFactory.createDefaultSparkMax(Settings.ShooterSystem.kAcceleratorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mHoodAngler = new Servo(Settings.ShooterSystem.kAnglerID);
        mTurret = new TalonSRX(Settings.ShooterSystem.kTurretID);

        mTurretGyro = new PigeonIMU(Settings.ShooterSystem.kTurretGyroID);
        mShooterPid = new PIDController(Settings.ShooterSystem.kTurretAngleLockGains, 0, 1, Settings.kControlLoopPeriod );
        mLimelight = pLimelight;
        mData = pData;
    }

    public enum EShooterState {
        SHOOT,
        STOP
    }

    public enum EAcceleratorState {
        FEED,
        STOP
    }

    private double calcSpeedFromDistance(double distance) { return 0.09 * Math.pow(distance, 2) + 8.0; }
    private double calcAngleFromDistance(double distance, double height) { return Math.atan(height / distance); }

    private boolean isMaxVelocity() { return mShooter.getEncoder().getVelocity() >= Settings.kMaxNeoVelocity; }
    private boolean targetValid() { return mData.limelight.get(ETargetingData.ty) != null; }

    private EShooterState getShooterState() { return mShooterState; }

    public void setShooterState(EShooterState pShooterState) { mShooterState = pShooterState; }
    public void hoodAngle() { desiredHoodAngle = targetValid() ? calcAngleFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget), mData.limelight.get(ETargetingData.ty)) : 60; }

    public void shoot(double pTime) {
        desiredAcceleratorVelocity = isMaxVelocity() ? Settings.kAcceleratorTargetVelocity : 0;
        desiredShooterVelocity = targetValid() ? calcSpeedFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget)) : mShooterPid.calculate(Settings.kMaxNeoVelocity, pTime - mPreviousTime);
        mPreviousTime = pTime;
    }

    public void turretTurn(boolean isGyro) {
        robotHeading = isGyro ?  mTurretGyro.getCompassHeading() : mData.limelight.get(ETargetingData.tx);
        desiredTurretVelocity = isGyro ? -2 * robotHeading : -10 * robotHeading;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
    }

    @Override
    public void readInputs(double pNow) {
        mData.flywheel.set(EShooterData.CURRENT_FLYWHEEL_STATE, (double) getShooterState().ordinal());
        mData.flywheel.set(EShooterData.TARGET_FLYWHEEL_STATE, (double) mShooterState.ordinal());
        mData.flywheel.set(EShooterData.CURRENT_FLYWHEEL_VELOCITY, mShooter.getEncoder().getVelocity());
        mData.flywheel.set(EShooterData.CURRENT_LIMELIGHT_TARGET, (double) mTrackingType.ordinal());
        mData.flywheel.set(EShooterData.CURRENT_HOOD_ANGLE, desiredHoodAngle);
        mData.flywheel.set(EShooterData.CURRENT_TURRET_HEADING, robotHeading );
        mData.flywheel.set(EShooterData.CURRENT_TURRET_VELOCITY, desiredTurretVelocity );
    }

    @Override
    public void setOutputs(double pNow) {
        mAccelerator.set(desiredAcceleratorVelocity);
        mHoodAngler.setAngle(desiredHoodAngle);
        mShooter.set(desiredShooterVelocity);
        mTurret.set(ControlMode.Velocity, desiredTurretVelocity);
    }

    @Override
    public void shutdown(double pNow) {

    }
}
