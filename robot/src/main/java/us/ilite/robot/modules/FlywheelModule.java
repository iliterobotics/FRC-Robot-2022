package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {
    //Hood Servo Gear Ratio
    // 32 / 20 * 14 / 360

    private CANSparkMax mFlywheelFeeder;
    private TalonFX mFlywheelFalcon;
    private Servo mHoodAngler;
    private Potentiometer mHoodPot;

    private double potentiometerTurns;

    private final double kHoodGearRatio = 14d / 225d;
    private final double kHoodConstant = kHoodGearRatio * 10;
    private final int kBaseHoodAngle = 60;
    private final int kFlywheelFalconPIDSlot = 0;
    private final int kMaxFalconVelocity = 6380;

    public FlywheelModule() {
        mFlywheelFalcon = new TalonFX(50);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(9 , CANSparkMaxLowLevel.MotorType.kBrushless);
        mHoodAngler = new Servo(9);

        mHoodPot = new AnalogPotentiometer(0);

        mFlywheelFalcon.config_kP(kFlywheelFalconPIDSlot, 0.25);
        mFlywheelFalcon.config_kI(kFlywheelFalconPIDSlot, 0);
        mFlywheelFalcon.config_kD(kFlywheelFalconPIDSlot, 0);
        mFlywheelFalcon.config_kF(kFlywheelFalconPIDSlot, 0);
    }

    private boolean isMaxVelocity() {
        return mFlywheelFalcon.getSelectedSensorVelocity() >= 2000;
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

    private double potentiometerBasedAngle() {
//        double distanceFromTarget;
//        if (targetValid()) {
//            distanceFromTarget = Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET);
//
//        }
        return 60 - kHoodGearRatio * Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_POTENTIOMETER_READING);
//        return kHoodConstant - kHoodGearRatio * Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_POTENTIOMETER_READING);
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);

        SmartDashboard.putNumber("Flywheel Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Hood Angle", potentiometerBasedAngle());

        Shuffleboard.addEventMarker("Flywheel Current Velocity", EventImportance.kHigh);
    }

    @Override
    public void readInputs(double pNow) {
        Distance distanceFromTarget = Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, DriveModule.Conversions.ticksPer100msToRotationsPerSecond(mFlywheelFalcon.getSelectedSensorVelocity()));

//        if (isMaxVelocity()) {
//            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 1);
//        } else {
//            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 0);
//        }

        if (targetValid() && Robot.DATA.limelight.get( ELimelightData.CURRENT_PIPELINE ) == 1 ) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 60);
        }

        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_POTENTIOMETER_READING, mHoodPot.get());
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelFalcon.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));
        mHoodAngler.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE));
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }
}