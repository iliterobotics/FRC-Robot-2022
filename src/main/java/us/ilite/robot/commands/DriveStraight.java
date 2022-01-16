package us.ilite.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;

import static us.ilite.common.types.sensor.EGyro.HEADING_DEGREES;

import static us.ilite.common.types.drive.EDriveData.*;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ECommonControlMode;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.modules.DriveModule;
import us.ilite.robot.modules.DriveMessage;


/**
 * A drivetrain straight command implementing heading control based on both angular velocity and angle.
 * Acceleration/deceleration can be controlled using either a custom implementation relying on
 * % output or the Talon's motion magic control mode.
 */
public class DriveStraight implements ICommand {

    private final EDriveControlMode mDriveControlMode;

    private Distance mDistanceToDrive;
    private Distance mInitialDistance;
    private Rotation2d mTargetHeading = null;

    private double mDrivePercentOutput = 0.3;
    private double mAllowableDistanceError = 3.0;
    private double mRampDistance = 120.0;
    private double mLastTime = 0.0;
    private double mStartTime = 0.0;
    private PIDController mHeadingController = new PIDController(
            DriveModule.kDriveHeadingGains, -180.0, 180.0, Settings.kControlLoopPeriod);

    private ProfiledPIDController mDistanceController = DriveModule.dPID.getPIDGains().generateController();

    public DriveStraight(EDriveControlMode pDriveControlMode, Distance pDistanceToDrive) {
        mDistanceToDrive = pDistanceToDrive;
        mDriveControlMode = pDriveControlMode;
        mDistanceController.setGoal(mDistanceToDrive.inches());
    }

    /**
     * Indicates whether we use velocity control or pure % output for drivebase outputs.
     */
    public enum EDriveControlMode {
//        MOTION_MAGIC(ECommonControlMode.MOTION_PROFILE),
//        PERCENT_OUTPUT(ECommonControlMode.PERCENT_OUTPUT);
//        public final ECommonControlMode kMotorControlMode;
//        EDriveControlMode(ECommonControlMode pMotorControlMode) {
//            kMotorControlMode = pMotorControlMode;
//        }
    }

    @Override
    public void init(double pNow) {
        // Set target heading to current heading if setTargetHeading() wasn't called manually
        if(mTargetHeading == null) {
            // TODO - was this inverted?
            mTargetHeading = Rotation2d.fromDegrees(Robot.DATA.imu.get(HEADING_DEGREES));
        }
        mInitialDistance = getAverageDriveDistance();
        mLastTime = pNow;
        mStartTime = pNow;

        mHeadingController.setContinuous(true);
        mHeadingController.setOutputRange(-1.0, 1.0);
        mHeadingController.reset();
    }

    @Override
    public boolean update(double pNow) {

        double turn = mHeadingController.calculate(getHeading().degrees(), pNow - mLastTime);
        // TODO - the units here are probably incorrect
        double throttle = mDistanceController.calculate(getAverageDriveDistance().inches());

        if(mDistanceController.atSetpoint()) {
            return true;
        } else {
            DriveMessage d = new DriveMessage().throttle(throttle).turn(turn).normalize();
            Robot.DATA.drivetrain.set(NEUTRAL_MODE, ECommonNeutralMode.BRAKE);
            Robot.DATA.drivetrain.set(DESIRED_THROTTLE_PCT, d.getThrottle());
            Robot.DATA.drivetrain.set(DESIRED_TURN_PCT, d.getTurn());
            mLastTime = pNow;
            return false;
        }

    }

    protected Angle getHeading() {
        return Angle.fromDegrees(Robot.DATA.imu.get(HEADING_DEGREES));
    }

    @Override
    public void shutdown(double pNow) {

    }

    private Distance getAverageDriveDistance() {
        return Distance.fromFeet(
                (Robot.DATA.drivetrain.get(L_ACTUAL_POS_FT) +
                        Robot.DATA.drivetrain.get(R_ACTUAL_POS_FT)) / 2.0);
    }

    private Distance getAverageDistanceTraveled() {
        return Distance.fromInches(getAverageDriveDistance().inches() - mInitialDistance.inches());
    }

    public DriveStraight setDistanceToDrive(Distance pDistanceToDrive) {
        mDistanceToDrive = pDistanceToDrive;
        return this;
    }

    public DriveStraight setTargetHeading(Rotation2d pTargetHeading) {
        mTargetHeading = pTargetHeading;
        mHeadingController.setSetpoint(mTargetHeading.getDegrees());
        return this;
    }

    public DriveStraight setHeadingGains(ProfileGains pHeadingControlGains) {
        mHeadingController.setPIDGains(pHeadingControlGains);
        return this;
    }

    public DriveStraight setDrivePercentOutput(double pDrivePercentOutput) {
        mDrivePercentOutput = pDrivePercentOutput;
        return this;
    }

}