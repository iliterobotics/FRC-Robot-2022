package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {

    private CANSparkMax mFlywheelFeeder;
    private CANSparkMax mTurret;
    private TalonFX mFlywheelMasterOne;
    private TalonFX mFlywheelMasterTwo;
    private Servo mServo;

    private CANEncoder mTurretEncoder;
    private CANEncoder mFeederEncoder;
    //Add SparkMax later
    private PigeonIMU mTurretGyro;

    private double kAcceleratorThreshold = 0.15;

    public FlywheelModule() {
        mFlywheelMasterOne = new TalonFX(50);
        mFlywheelMasterTwo = new TalonFX(51);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(5 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurret = SparkMaxFactory.createDefaultSparkMax(16 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kPigeonIDForFlywheel);
        mFeederEncoder = mFlywheelFeeder.getEncoder();
        mTurretEncoder = mTurret.getEncoder();
        mServo = new Servo(9);
    }

    private boolean isMaxVelocity() {
        return mFlywheelMasterOne.getSelectedSensorVelocity() >= kAcceleratorThreshold;
    }

    private boolean targetValid(ELimelightData pLimelightData) {
        return Robot.DATA.limelight.isSet(pLimelightData);
    }

    private double calcSpeedFromDistance(Distance pDistance) {
        return 7.2E-3 * Math.pow(pDistance.inches(), 3) - 0.209 * Math.pow(pDistance.inches(), 2) + 6.31 * pDistance.inches() + 227;
    }

    private double calcAngleFromDistance(Distance pDistance) {
        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
                + 0.157 * Math.pow(pDistance.inches(), 2)
                + 2.94 * pDistance.inches()
                + 68.2;
    }

    private double calcAngle() {
        return Math.atan(Robot.DATA.limelight.get(ELimelightData.CALC_TARGET_Y) / Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
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
        if (isMaxVelocity()) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 1.0);
        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 0.0);
        }
        if (Robot.DATA.limelight.isSet(ELimelightData.TY)) {

        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 0.25);
        }
        if (targetValid(ELimelightData.TY)) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 0.25);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 60);
        }
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMasterOne.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelMasterTwo.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));
        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY ) );
        mTurret.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_VELOCITY));
        mServo.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE));
    }


    @Override
    public void shutdown(double pNow) {

    }

}