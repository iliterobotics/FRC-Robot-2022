package us.ilite.robot.commands;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Conversions;
import us.ilite.common.lib.util.Utils;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.sensor.EGyro.YAW_DEGREES;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.hardware.ECommonControlMode;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.modules.Drive;
import us.ilite.robot.modules.DriveMessage;


/**
 * A drive straight command implementing heading control based on both angular velocity and angle.
 * Acceleration/deceleration can be controlled using either a custom implementation relying on
 * % output or the Talon's motion magic control mode.
 */
public class DriveStraight implements ICommand {

    private final ILog mLog = Logger.createLog(DriveStraight.class);

    private final Drive mDrive;
    private final Data mData;

    private final EDriveControlMode mDriveControlMode;

    private double mDistanceToDrive;
    private double mInitialDistance;
    private Rotation2d mTargetHeading = null;

    private double mDrivePercentOutput = 0.3;
    private double mAllowableDistanceError = 3.0;
    private double mRampDistance = 120.0;
    private double mLastTime = 0.0;
    private double mStartTime = 0.0;
    private PIDController mHeadingController = new PIDController(Settings.kDriveHeadingGains, -180.0, 180.0, Settings.kControlLoopPeriod);

    private ProfiledPIDController mDistanceController = Settings.Drive.kDistancePID.generateController();

    public DriveStraight(Drive pDrive, Data pData, EDriveControlMode pDriveControlMode, double pDistanceToDrive) {
        mDrive = pDrive;
        mData = pData;
        mDistanceToDrive = pDistanceToDrive;
        mDriveControlMode = pDriveControlMode;
        mDistanceController.setGoal(mDistanceToDrive);
    }

    /**
     * Indicates whether we use velocity control or pure % output for drivebase outputs.
     */
    public enum EDriveControlMode {
        MOTION_MAGIC(ECommonControlMode.MOTION_PROFILE),
        PERCENT_OUTPUT(ECommonControlMode.PERCENT_OUTPUT);

        public final ECommonControlMode kMotorControlMode;

        EDriveControlMode(ECommonControlMode pMotorControlMode) {
            kMotorControlMode = pMotorControlMode;
        }
    }

    @Override
    public void init(double pNow) {
        // Set target heading to current heading if setTargetHeading() wasn't called manually
        if(mTargetHeading == null) {
            // TODO - was this inverted?
            mTargetHeading = Rotation2d.fromDegrees(mData.imu.get(YAW_DEGREES));
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

        double turn = mHeadingController.calculate(mData.imu.get(YAW_DEGREES), pNow - mLastTime);
        // TODO - the units here are probably incorrect
        double throttle = mDistanceController.calculate(getAverageDriveDistance());

        if(mDistanceController.atSetpoint()) {

            // Stop drivebase
            mDrive.setDriveMessage(DriveMessage.kNeutral);

            mLastTime = pNow;
            return true;
        } else {
            DriveMessage driveMessage = new DriveMessage().throttle(throttle).turn(turn).normalize();
            mDrive.setDriveMessage(driveMessage);
            mLastTime = pNow;

//            Data.kSmartDashboard.putDouble("Angle Error", mHeadingController.getError());
//            Data.kSmartDashboard.putDouble("PID Output", mHeadingController.getOutput());
//            Data.kSmartDashboard.putDouble("Angle", mData.imu.get(EGyro.YAW_DEGREES));
//            Data.kSmartDashboard.putDouble("Target Angle", mTargetHeading.getDegrees());
//            Data.kSmartDashboard.putDouble("Linear Output", linearOutput);
//            Data.kSmartDashboard.putDouble("Distance Target", mDistanceToDrive);
//            Data.kSmartDashboard.putDouble("Distance Error", distanceError);

            return false;
        }

    }

    @Override
    public void shutdown(double pNow) {

    }

    private double getAverageDriveDistance() {
        return (mData.drive.get(EDriveData.LEFT_POS_INCHES) + mData.drive.get(EDriveData.RIGHT_POS_INCHES)) / 2.0;
    }

    private double getAverageDistanceTraveled() {
        return getAverageDriveDistance() - mInitialDistance;
    }

    public DriveStraight setDistanceToDrive(double pDistanceToDrive) {
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
