package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.EFlywheelData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelModule extends Module {

    private Data mData;
    private CANSparkMax mShooter;
    private Servo mAngler;
    private TalonSRX mTurret;
    private CANSparkMax mAccelerator;
    private EShooterState mShooterState;
    private Limelight mLimelight;

    private double mDesiredOutput;
    private double distanceToTarget;

    public FlywheelModule(Data pData) {
            mShooter = SparkMaxFactory.createDefaultSparkMax(Settings.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAccelerator = SparkMaxFactory.createDefaultSparkMax(Settings.kAccelerator, CANSparkMaxLowLevel.MotorType.kBrushless);
        mAngler = new Servo(Settings.kAnglerID);
        mTurret = new TalonSRX(Settings.kTurretID);
        mData = pData;


//        mShooter.getPIDController().setP(Settings.Shooter.kShooterPGain);
//        mShooter.getPIDController().setFF(Settings.Shooter.kShooterFF);
//        mShooter.getPIDController().setReference(mDesiredOutput, ControlType.kVelocity);
    }

    public enum EShooterState {
        SHOOT,
        STOP
    }

 //   public boolean isMaxVelocity() { return mShooter.getEncoder().getVelocity() >= Settings.Shooter.kMaxShooterVelocity; }

    public void setShooterState( EShooterState pShooterState ) { mShooterState = pShooterState; }

    public EShooterState getShooterState() { return mShooterState; }

    public void shoot() { mShooter.set(mDesiredOutput); }

//    public void feed() { mAccelerator.set(Settings.Shooter.kMaxAcceleratorVelocity); }
//
    private void angle() {
        if ( mData.limelight.get(ETargetingData.ty) != null ) {
            distanceToTarget = mLimelight.calcTargetDistance(62); // inner port is 98.25
            mAngler.set(distanceToTarget / 15); // Under Review
        }
        else {
            //mAngler.setAngle(Settings.Shooter.kBaseAngle);
        }
    }

    public void turnTurret() {

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        setShooterState(EShooterState.STOP);
    }

    @Override
    public void readInputs(double pNow) {
        mData.flywheel.set(EFlywheelData.CURRENT_FLYWHEEL_STATE, (double) getShooterState().ordinal());
        mData.flywheel.set(EFlywheelData.TARGET_FLYWHEEL_STATE, (double) mShooterState.ordinal());
        mData.flywheel.set(EFlywheelData.CURRENT_FLYWHEEL_VELOCITY, mShooter.getEncoder().getVelocity());
    }

    @Override
    public void setOutputs(double pNow) {
        SmartDashboard.putNumber("Hood Angle", mAngler.getAngle());
    }

    @Override
    public void shutdown(double pNow) {
        setShooterState(EShooterState.STOP);
    }
}
