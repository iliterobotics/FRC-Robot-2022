package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {

    private CANSparkMax mFlywheelFeeder;
    private TalonFX mFlywheelFalcon;

    private double mPreviousTime;

    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
        mFlywheelFalcon = new TalonFX(50);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(9 , CANSparkMaxLowLevel.MotorType.kBrushless);

        mFlywheelFalcon.config_kP(kFlywheelFalconPIDSlot, 0.005);
        mFlywheelFalcon.config_kI(kFlywheelFalconPIDSlot, 0);
        mFlywheelFalcon.config_kD(kFlywheelFalconPIDSlot, 0);
    }

    private boolean isMaxVelocity() {
        return mFlywheelFalcon.getSelectedSensorVelocity() >= 2000;
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

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 60);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);

        SmartDashboard.putNumber("Flywheel Current Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Flywheel Target Velocity", Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));

        Shuffleboard.addEventMarker("Flywheel Current Velocity", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Flywheel Target Velocity", EventImportance.kHigh);
    }

    @Override
    public void readInputs(double pNow) {
        Distance distanceFromTarget = Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelFalcon.getSelectedSensorVelocity());

        if (isMaxVelocity()) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 1);
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 0);
        }

        if (targetValid(ELimelightData.TY) && Robot.DATA.limelight.get( ELimelightData.CURRENT_PIPELINE ) == 1 ) {
//            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 1250);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 60);
        }
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelFalcon.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));

        mPreviousTime = pNow;
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 60);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }
}