package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelModule extends Module {
    // Hood Servo Gear Ratio
    // 32 / 20 * 14 / 360

    public static class Conversions {

        private final double kConstant = 1;

        double ticksPer100msToRPM(double pTicks) {
            return kConstant * pTicks;
        }

    }

    private CANSparkMax mFlywheelFeeder;
    private TalonFX mFlywheelFalcon;
    private Servo mHoodAngler;
    private CANSparkMax mTurret;

    private Potentiometer mHoodPot;

    private PIDController mTurretPID;

    private CANEncoder mTurretEncoder;

    private double target;
    private double current;

    private final double kHoodGearRatio = 14d / 225d;
    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        mFlywheelFalcon = new TalonFX(50);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(0 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurret = SparkMaxFactory.createDefaultSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushless);
        mHoodAngler = new Servo(9);

        mTurretEncoder = mTurret.getEncoder();

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

    private double turretTurn() {
        double target = Robot.DATA.limelight.get(ELimelightData.TX);
        double current = Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_TURRET_POSITION);

        return -target;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_POSITION, 0);
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
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_TURRET_POSITION, mTurretEncoder.getVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_POTENTIOMETER_TURNS, mHoodPot.get());
    }

    @Override
    public void setOutputs(double pNow) {
        double current = Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_TURRET_POSITION);
        double target = Robot.DATA.flywheel.get(EShooterSystemData.TARGET_TURRET_POSITION);
        SmartDashboard.putNumber("Turret Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_TURRET_POSITION));
        mTurret.set(target);
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_POSITION, 0);
    }
}