package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.PIDGains;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;


public class FlywheelModule extends Module {
   // public static final double kAcceleratorThreshold = 0.14;

    private CANSparkMax mFlywheelFeeder;
    private CANSparkMax mTurret;
    private TalonFX mFlywheelMasterOne;
    private TalonFX mFlywheelMasterTwo;
    private Servo mServo;

    private CANEncoder mTurretEncoder;
    private CANEncoder mFeederEncoder;
    //Add SparkMax later
    private PigeonIMU mTurretGyro;

    public FlywheelModule() {
        mFlywheelMasterOne = new TalonFX(50);
        mFlywheelMasterTwo = new TalonFX(51);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(5 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurret = SparkMaxFactory.createDefaultSparkMax(16 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kPigeonIDForFlywheel);
        mFeederEncoder = mFlywheelFeeder.getEncoder();
        mTurretEncoder = mTurret.getEncoder();
    }
    public enum EFlywheelState{
        SHOOTING(1700),
        REVERSE(-1700),
        STOP(0);

        double velocity;
        EFlywheelState(double velocity){
            this.velocity = velocity;
        }
    }
    public enum EHoodState{
        //TODO find the states of this
    }
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelMasterOne.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FEEDER_VELOCITY , mFeederEncoder.getVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY , mTurretEncoder.getVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_SERVO_ANGLE, mServo.getAngle());
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMasterOne.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelMasterTwo.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));
        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY ) );
        mTurret.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_VELOCITY));
    }


    @Override
    public void shutdown(double pNow) {

    }

}