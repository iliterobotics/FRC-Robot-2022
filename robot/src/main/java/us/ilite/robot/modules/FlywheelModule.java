package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.lang3.Conversion;
import us.ilite.common.Distance;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.util.Conversions;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

public class FlywheelModule extends Module {
    // Hood Servo Gear Ratio
    // 32 / 20 * 14 / 360

    private CANSparkMax mFlywheelFeeder;
    private TalonFX mFlywheelFalcon;
    private Servo mHoodAngler;
    private TalonSRX mTurret;

    private Potentiometer mHoodPot;

    private PIDController mTurretPID;

    private double target;
    private double current;

    private final double kHoodGearRatio = 14d / 225d;
    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        mFlywheelFalcon = new TalonFX(50);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(0 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurret = TalonSRXFactory.createDefaultTalon(9);
        mHoodAngler = new Servo(9);

        mHoodPot = new AnalogPotentiometer(0);

        mFlywheelFalcon.config_kP(kFlywheelFalconPIDSlot, 0.005);
        mFlywheelFalcon.config_kI(kFlywheelFalconPIDSlot, 0);
        mFlywheelFalcon.config_kD(kFlywheelFalconPIDSlot, 0);
        mFlywheelFalcon.config_kF(kFlywheelFalconPIDSlot, 0);
    }

    private boolean isPastVelocity(double pVelocity) {
        // TODO Convert ticks per 100 ms to rpm
        return mFlywheelFalcon.getSelectedSensorVelocity() >= pVelocity;
    }

    private boolean targetValid() {
        return Robot.DATA.limelight.isSet(ELimelightData.TY);
    }

    private double calcSpeedFromDistance(Distance pDistance) {
        //TODO figure out necessity
        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
                - 0.209 * Math.pow(pDistance.inches(), 2)
                + 6.31 * pDistance.inches()
                + 227;
    }

    private double calcAngleFromDistance(Distance pDistance) {
        //TODO tuning
        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
                + 0.157 * Math.pow(pDistance.inches(), 2)
                + 2.94 * pDistance.inches()
                + 68.2;
    }

    private double hoodAnglePID(double pP, double pI, double pD) {
        // TODO Convert into an Angle Measurement
        double error = pP * (Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE) - Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_SERVO_ANGLE));
        double velocity = pD * mHoodAngler.getSpeed();
        mIntegral += pI * mHoodAngler.getSpeed();
        return error + mIntegral + velocity;
    }

    private void potentiometerPID() {
        double output = Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE) / 5;
    }

    private double turretTurn() {
        current = mTurret.getSelectedSensorVelocity();
        if (targetValid()) {
            target = Robot.DATA.selectedTarget.get(ELimelightData.TX);
        } else {
            target = 0;
        }
        return -target;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);

        SmartDashboard.putNumber("Flywheel Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Servo Current Angle", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_SERVO_ANGLE));
        SmartDashboard.putNumber("Turret Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_TURRET_VELOCITY));
        SmartDashboard.putNumber("Potentiometer Current Reading", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_POTENTIOMETER_TURNS));
        SmartDashboard.putNumber("Feeder Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FEEDER_VELOCITY));
    }

    @Override
    public void readInputs(double pNow) {
        Distance distanceFromTarget = Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));

//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, DriveModule.Conversions.ticksPer100msToRotationsPerSecond(mFlywheelFalcon.getSelectedSensorVelocity()));

        if (targetValid()) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 0);
        }
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_VELOCITY, mTurret.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_POTENTIOMETER_TURNS, mHoodPot.get());
    }

    @Override
    public void setOutputs(double pNow) {
//        mFlywheelFalcon.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
//        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));
//        mHoodAngler.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE));
        mTurret.set(ControlMode.PercentOutput, 0.25);
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }
}