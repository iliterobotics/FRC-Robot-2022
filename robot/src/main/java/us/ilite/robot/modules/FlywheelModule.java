package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.EFlywheelData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelModule extends Module {

    private Data mData;
    private CANSparkMax mShooter;
    private Servo mAngler;
    private TalonSRX mTurret;
    private CANSparkMax mAccelerator;
    private EShooterState mShooterState;
    private PIDController mShooterPid;
    private ETrackingType mTrackingType = ETrackingType.NONE;

    private final double kTurretTurnRate = 0.05;

    private double mPreviousTime= 0;
    private double desiredServoAngle = 0;
    private double distanceToTarget;
    private double shooterOutput = 0;
    private double mTurretOutput;

    public FlywheelModule(Data pData) {
        mShooter = SparkMaxFactory.createDefaultSparkMax(Settings.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAccelerator = SparkMaxFactory.createDefaultSparkMax(Settings.kAcceleratorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAngler = new Servo(Settings.kAnglerID);
        mTurret = new TalonSRX(Settings.kTurretID);
        mShooterPid = new PIDController(Settings.kTurretAngleLockGains, 0, 1, Settings.kControlLoopPeriod );
        mData = pData;
    }

    public enum EShooterState {
        SHOOT,
        STOP
    }

    public boolean isMaxVelocity() { return mShooter.getEncoder().getVelocity() >= 5000; }

    public boolean targetValid() { return mData.limelight.get(ETargetingData.ty) != null; }

    public void setShooterState( EShooterState pShooterState ) { mShooterState = pShooterState; }

    public EShooterState getShooterState() { return mShooterState; }

    private void hoodAngle() {
        if ( mData.flywheel.get(EFlywheelData.CURRENT_LIMELIGHT_TARGET).equals((double)ETrackingType.TARGET.ordinal())) {
            if (targetValid()) {
                double targetHeight = mData.limelight.get(ETargetingData.ty);
                desiredServoAngle = targetHeight * 90;
            }
            desiredServoAngle = 45;
        }
        mAngler.set(desiredServoAngle);
    }

    private void shoot(double pTime) {
        if ( isMaxVelocity() ) {
            mAccelerator.set(0.75);
        }
        if ( targetValid() ) {
            if (mData.flywheel.get(EFlywheelData.CURRENT_FLYWHEEL_STATE).equals((double) EShooterState.SHOOT.ordinal())) {
                mShooter.set(calcSpeedFromDistance(mData.limelight.get(ETargetingData.calcDistToTarget)));
            }
        }
        else if ( mData.flywheel.get(EFlywheelData.CURRENT_FLYWHEEL_STATE).equals((double)EShooterState.SHOOT.ordinal()) ){
            mShooterPid.calculate(2000, pTime - mPreviousTime);
        }
        mPreviousTime = pTime;
    }

    private double calcSpeedFromDistance(double distance) {
        return distance / 200 * Settings.kNeoMaxVelocity;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        mData.flywheel.set(EFlywheelData.CURRENT_FLYWHEEL_STATE, (double) getShooterState().ordinal());
        mData.flywheel.set(EFlywheelData.TARGET_FLYWHEEL_STATE, (double) mShooterState.ordinal());
        mData.flywheel.set(EFlywheelData.CURRENT_FLYWHEEL_VELOCITY, mShooter.getEncoder().getVelocity());
        mData.flywheel.set(EFlywheelData.CURRENT_LIMELIGHT_TARGET, (double) mTrackingType.ordinal() );
    }

    @Override
    public void setOutputs(double pNow) {
        shoot(pNow);
    }

    @Override
    public void shutdown(double pNow) {
        setShooterState(EShooterState.STOP);
    }
}
