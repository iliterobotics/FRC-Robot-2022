package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

//    private CANSparkMax mFlywheelFeeder;
//    private CANSparkMax mTurret;
    private TalonFX mFlywheelMasterOne;
    private PIDController mFlywheelPID;
    private ProfileGains mFlywheelGains = new ProfileGains();

    private ILog mLog = Logger.createLog(this.getClass());
//    private Servo mServo;

//    private CANEncoder mTurretEncoder;
//    private CANEncoder mFeederEncoder;
    //Add SparkMax later
//    private PigeonIMU mTurretGyro;

    private double kAcceleratorThreshold = 0.15;
    private double mPreviousTime;

    public FlywheelModule() {
        mFlywheelMasterOne = new TalonFX(50);
        mFlywheelGains.p(0.03);
        mFlywheelGains.i(0);
        mFlywheelGains.d(0.03);
        mFlywheelPID = new PIDController(mFlywheelGains, 0, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY), Settings.kControlLoopPeriod);
//        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(5 , CANSparkMaxLowLevel.MotorType.kBrushless);
//        mTurret = SparkMaxFactory.createDefaultSparkMax(16 , CANSparkMaxLowLevel.MotorType.kBrushless);
//        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kPigeonIDForFlywheel);
//        mFeederEncoder = mFlywheelFeeder.getEncoder();
//        mTurretEncoder = mTurret.getEncoder();
//        mServo = new Servo(9);
    }

    private boolean isMaxVelocity() {
        return mFlywheelMasterOne.getSelectedSensorVelocity() >= kAcceleratorThreshold;
    }

    private boolean targetValid(ELimelightData pLimelightData) {
        return Robot.DATA.limelight.isSet(pLimelightData);
    }

    private double calcSpeedFromDistance(Distance pDistance) {
        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
                - 0.209 * Math.pow(pDistance.inches(), 2)
                + 6.31 * pDistance.inches()
                + 227;
    }

    private double calcAngleFromDistance(Distance pDistance) {
        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
                + 0.157 * Math.pow(pDistance.inches(), 2)
                + 2.94 * pDistance.inches()
                + 68.2;
    }

    private double calcAngleFromTan() {
        return Math.atan(Robot.DATA.limelight.get(ELimelightData.CALC_TARGET_Y) / Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelMasterOne.getSelectedSensorVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FEEDER_VELOCITY , mFeederEncoder.getVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY , mTurretEncoder.getVelocity());
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_SERVO_ANGLE, mServo.getAngle());
        if (isMaxVelocity()) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 1.0);
        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 0.0);
        }
        if (targetValid(ELimelightData.TY)) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
        }
        else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 2250);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 60);
        }
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMasterOne.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mPreviousTime = pNow;
        SmartDashboard.putNumber("Flywheel PID Output", mFlywheelPID.getOutput());
        SmartDashboard.putNumber("Flywheel PID Error", mFlywheelPID.getError());
        SmartDashboard.putNumber("Flywheel PID Setpoint", mFlywheelPID.getSetpoint());
        mLog.error("----------------------------------------Flywheel PID Gains: " + mFlywheelPID.getPIDGains());
    }


    @Override
    public void shutdown(double pNow) {

    }

}