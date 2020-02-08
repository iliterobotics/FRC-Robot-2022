package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.NetworkTablesConstantsBase;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;


public class FlywheelModule extends Module {

    private CANSparkMax mFlywheelFeeder;
    private TalonFX mFlywheelMasterOne;
    private PIDController mFlywheelPID;
    private ProfileGains mFlywheelGains = new ProfileGains();

    private double mPreviousTime;

    public FlywheelModule() {
        mFlywheelGains.p(0.5);
        mFlywheelGains.i(0.2);
        mFlywheelGains.d(0.3);
        mFlywheelGains.generateController();
        mFlywheelMasterOne = new TalonFX(50);
        mFlywheelPID = new PIDController(mFlywheelGains, 0, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY), Settings.kControlLoopPeriod);
        mFlywheelFeeder = SparkMaxFactory.createDefaultSparkMax(9 , CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    private boolean isMaxVelocity() {
        return mFlywheelMasterOne.getSelectedSensorVelocity() >= 2000;
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
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 60);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelMasterOne.getSelectedSensorVelocity());

        if (isMaxVelocity()) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 1);
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_IS_MAX_VELOCITY, 0);
        }

        if (targetValid(ELimelightData.TY)) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET))));
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 2750);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 60);
        }
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMasterOne.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));

        SmartDashboard.putNumber("Flywheel Target Velocity", Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Flywheel Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        SmartDashboard.putNumber("Feeder Velocity", Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FEEDER_VELOCITY));
        SmartDashboard.putNumber("Flywheel Gains P", mFlywheelGains.P);
        SmartDashboard.putNumber("Flywheel Gains I", mFlywheelGains.I);
        SmartDashboard.putNumber("Flywheel Gains D", mFlywheelGains.D);

        Shuffleboard.addEventMarker("Flywheel Target Velocity", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Flywheel Velocity", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Feeder Velocity", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Flywheel Gains P", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Flywheel Gains I", EventImportance.kHigh);
        Shuffleboard.addEventMarker("Flywheel Gains D", EventImportance.kHigh);

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