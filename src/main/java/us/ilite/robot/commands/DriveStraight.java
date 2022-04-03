package us.ilite.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.lib.control.ProfileGains;

import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.modules.NeoDriveModule;


/**
 * A drivetrain straight command implementing heading control based on both angular velocity and angle.
 * Acceleration/deceleration can be controlled using either a custom implementation relying on
 * % output or the Talon's motion magic control mode.
 */
public class DriveStraight implements ICommand {


    private Distance mDistanceToDrive;
    private Distance mInitialDistance;
    private Rotation2d mTargetHeading = null;

    private double mDrivePercentOutput = 0.3;
    private double mAllowableDistanceError = 3.0;
    private double mRampDistance = 120.0;
    private double mLastTime = 0.0;
    private double mStartTime = 0.0;
    private PIDController mHeadingController = NeoDriveModule.kTurnToProfileGains.generateWPIPIDController();
    private ProfiledPIDController mDistanceController = NeoDriveModule.kPositionGains.generateController();

    public DriveStraight( Distance pDistanceToDrive) {
        mDistanceToDrive = pDistanceToDrive;
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
        if (mTargetHeading == null) {
//            mTargetHeading = Rotation2d.fromDegrees(Robot.DATA.drivetrain.get(ACTUAL_HEADING_DEGREES));
            mTargetHeading = Rotation2d.fromDegrees(getHeading().degrees());
        }
        mInitialDistance = getAverageDriveDistance();
        mLastTime = pNow;
        mStartTime = pNow;
        mHeadingController.setSetpoint(mTargetHeading.getDegrees());

        mHeadingController.reset();
    }

    @Override
    public boolean update(double pNow) {

        double turn = mHeadingController.calculate(getHeading().degrees());
//        // TODO - the units here are probably incorrect
        double throttle = mDistanceController.calculate(getAverageDriveDistance().inches());
        SmartDashboard.putNumber("Distance Error", mDistanceController.getPositionError());
//
        SmartDashboard.putNumber("AUTON Turn Output", turn);
        SmartDashboard.putNumber("AUTON Heading Degrees", Robot.DATA.drivetrain.get( ACTUAL_HEADING_DEGREES ));
        SmartDashboard.putNumber("AUTON Drive Error", mDistanceController.getPositionError() );
        if(mDistanceController.atSetpoint()) {
            return true;
        } else {
            Robot.DATA.drivetrain.set(NEUTRAL_MODE, ECommonNeutralMode.BRAKE);
            Robot.DATA.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
            Robot.DATA.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
            Robot.DATA.drivetrain.set(DESIRED_TURN_PCT, turn);
            mLastTime = pNow;
            return false;
        }


    }

    protected Angle getHeading() {
        return Angle.fromDegrees(Robot.DATA.drivetrain.get(ACTUAL_HEADING_DEGREES));
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
        mHeadingController.setPID(pHeadingControlGains.P, pHeadingControlGains.I, pHeadingControlGains.D);
        return this;
    }

    public DriveStraight setDrivePercentOutput(double pDrivePercentOutput) {
        mDrivePercentOutput = pDrivePercentOutput;
        return this;
    }

}