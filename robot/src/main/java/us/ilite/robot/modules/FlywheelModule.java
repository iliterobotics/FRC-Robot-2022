package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {
    public static final double kAcceleratorThreshold = 0.15;
    private PigeonIMU mTurretGyro;

    private final ProfileGains kShooterGains = new ProfileGains().p(0.0005);

    private double mPreviousTime;

//    public CANSparkMax mFlywheelMaster;
//    private Servo mHoodAngler;
//    private TalonSRX mTurret;
//    private TalonSRX mAccelerator;
    private TalonFX mFlywheelMaster;
//    private CANEncoder mShooterEncoder;

    private PIDController mShooterPID;
    private ELimelightData mTrackingType;

    private double desiredFlywheelVelocity = 0;
    private double kShooterTargetVelocity = 1885;

    public FlywheelModule() {
//        mFlywheelMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kShooterID, CANSparkMaxLowLevel.MotorType.kBrushless);
//        mAccelerator = new TalonSRX(Settings.Hardware.CAN.kAcceleratorID);
//        mHoodAngler = new Servo(Settings.Hardware.CAN.kAnglerID);
//        mTurret = new TalonSRX(Settings.Hardware.CAN.kTurretID);
//        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kTurretGyroID);
//        mShooterEncoder = mFlywheelMaster.getEncoder();
//        mShooterPID = new PIDController(kShooterGains, 0, 0.5, 0);
        mFlywheelMaster = new TalonFX(51);
    }

    public double calcSpeedFromDistance(Distance distance) {
        return 0.09 * Math.pow(distance.inches(), 2) + 8.0;
    } // Need recalculation

    public Angle calcAngleFromDistance(Distance distance, Distance height) {
        return Angle.fromDegrees(Math.atan(height.inches() / distance.inches()));
    }

    public boolean isMaxVelocity() {
        return mFlywheelMaster.getSelectedSensorVelocity() >= kAcceleratorThreshold;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, (double) mFlywheelMaster.getSelectedSensorVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY, (double) mTurret.getSelectedSensorVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_HOOD_ANGLE, mHoodAngler.getAngle());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_ACCELERATOR_VELOCITY, (double)mAccelerator.getSelectedSensorVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_MODE, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_MODE));
//        Robot.DATA.flywheel.set(EShooterSystemData.DESIRED_FLYWHEEL_VELOCITY, desiredFlywheelVelocity);


    }

    @Override
    public void setOutputs(double pNow) {
//        desiredFlywheelVelocity = mShooterPID.calculate(kShooterTargetVelocity, pNow - mPreviousTime);
        if ( isMaxVelocity() ) {
//            mAccelerator.set(ControlMode.PercentOutput, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_ACCELERATOR_VELOCITY));
        }
        else {
//            mAccelerator.set(ControlMode.PercentOutput, 0.0);
        }
        mFlywheelMaster.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
//        mTurret.set(ControlMode.PercentOutput, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_VELOCITY));
//        mPreviousTime = pNow;
    }

    @Override
    public void shutdown(double pNow) {

    }
}