package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {
    private CANSparkMax mShooterNeo;
    private Servo mHoodAngler;
    private TalonSRX mTurret;
    //private CANSparkMax mAccelerator;
    private TalonSRX mAccelerator;
    private EShooterState mShooterState = EShooterState.STOP;
    private EHoodState mHoodState = EHoodState.BASE;
    private EAcceleratorState mAcceleratorState = EAcceleratorState.STOP;
    private ETrackingType mTrackingType = ETrackingType.NONE;
    private ETurretMode mTurretMode = ETurretMode.GYRO;

    public FlywheelModule() {
        mShooterNeo = SparkMaxFactory.createDefaultSparkMax(Settings.ShooterSystem.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
//        mAccelerator = SparkMaxFactory.createDefaultSparkMax(Settings.ShooterSystem.kAcceleratorID, CANSparkMaxLowLevel.MotorType.kBrushless);
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
        BASE,
        ADJUSTABLE
    }

    public double calcSpeedFromDistance(double distance) { return 0.09 * Math.pow(distance, 2) + 8.0; }
    public double calcAngleFromDistance(double distance, double height) { return Math.atan(height / distance); }

    public boolean isMaxVelocity() { return mShooterNeo.getEncoder().getVelocity() >= Settings.kMaxNeoVelocity; }
    public boolean targetValid() { return Robot.DATA.limelight.get(ETargetingData.ty) != null; }

    public void setShooterState(EShooterState pShooterState) { mShooterState = pShooterState; }
    private void reset() { setShooterState(EShooterState.STOP); }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        reset();
    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_STATE, (double)mShooterState.ordinal());
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, (double)EShooterState.SHOOT.ordinal());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mShooterNeo.getEncoder().getVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_LIMELIGHT_TARGET, Robot.DATA.limelight.get(ETargetingData.targetOrdinal));
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_LIMELIGHT_TARGET, (double) mTrackingType.ordinal());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_HOOD_ANGLE, mHoodAngler.getAngle());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY, (double) mTurret.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_MODE, (double) mTurretMode.ordinal());
    }

    @Override
    public void setOutputs(double pNow) {
        if ( isMaxVelocity() ) {
            mAccelerator.set(ControlMode.PercentOutput, Settings.ShooterSystem.kAcceleratorTargetVelocity);
        }
        else {
            mAccelerator.set(ControlMode.PercentOutput, 0.0);
        }
        mHoodAngler.setAngle(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_HOOD_ANGLE));
        Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY);
        mTurret.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_VELOCITY));
    }

    @Override
    public void shutdown(double pNow) {
        reset();
    }
}