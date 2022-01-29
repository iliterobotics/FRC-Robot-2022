package us.ilite.robot.controller;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.modules.VioletDriveModule;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class BaseAutonController extends AbstractController {

    private final Timer mTimer;
    private final boolean mUsePid;

    //TODO figure out way to store trajectories and switch between them
    private Trajectory mTrajectory;

    private final RamseteController mFollower;
    private final SimpleMotorFeedforward mFeedforward;
    private DifferentialDriveKinematics mDriveKinematics;
    private final PIDController mLeftController;
    private final PIDController mRightController;
    private DifferentialDriveWheelSpeeds mPrevSpeeds;
    private double mPrevTime;

    public BaseAutonController() {
        mUsePid = true;
        mFollower = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kP, Settings.kA);
        mRightController = new PIDController(Settings.kP, 0, 0);
        mLeftController = new PIDController(Settings.kP, 0, 0);

        mTimer = new Timer();
        mDriveKinematics = new DifferentialDriveKinematics(Settings.kTrackwidthMeters);


    }
    public void initialize() {
        mTimer.reset();
        mTimer.start();
        mLeftController.reset();
        mRightController.reset();
        mPrevTime = -1;
        mTrajectory = TrajectoryCommandUtils.getTrajectory();
        var initialState = mTrajectory.sample(0);
        mPrevSpeeds =
                mDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        mTimer.reset();
        mTimer.start();
        if (mUsePid) {
            mLeftController.reset();
            mRightController.reset();
        }

    }
    @Override
    protected void updateImpl() {
        execute();
    }

    public void execute() {
        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
        double curTime = mTimer.get();
        double dt = curTime - mPrevTime;

        if (mPrevTime < 0) {
            updateDriveTrain(0,0);
            mPrevTime = curTime;
            return;
        }
        Pose2d robotPose = getRobotPose();
        var targetWheelSpeeds = getTargetWheelSpeeds(curTime, robotPose);

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (mUsePid) {
            double leftFeedforward =
                    mFeedforward.calculate(
                            leftSpeedSetpoint, (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    mFeedforward.calculate(
                            rightSpeedSetpoint, (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt);
            double actualLeftSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
            double actualRightSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s));

//            System.out.println("BaseAutonController: actualSpeeds: ["+actualLeftSpeed+","+actualRightSpeed+"]");

            DifferentialDriveWheelSpeeds actualSpeeds = new DifferentialDriveWheelSpeeds(actualLeftSpeed, actualRightSpeed);
            leftOutput =
                    leftFeedforward
                            + mLeftController.calculate(actualSpeeds.leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput =
                    rightFeedforward
                            + mRightController.calculate(
                            actualSpeeds.rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        updateDriveTrain(leftOutput, rightOutput);
        mPrevSpeeds = targetWheelSpeeds;
        mPrevTime = curTime;
        updateDriveTrain(leftOutput, rightOutput);
    }

    private DifferentialDriveWheelSpeeds getTargetWheelSpeeds(double curTime, Pose2d robotPose) {
        return mDriveKinematics.toWheelSpeeds(
                mFollower.calculate(robotPose, mTrajectory.sample(curTime)));
    }

    private Pose2d getRobotPose() {
        Rotation2d r2d = new Rotation2d(Units.degrees_to_radians(db.drivetrain.get(EDriveData.DELTA_HEADING)));
        Pose2d robotPose = new Pose2d(db.drivetrain.get(EDriveData.GET_X_OFFSET), db.drivetrain.get(EDriveData.GET_Y_OFFSET), r2d);
        return robotPose;
    }

    private void updateDriveTrain(double leftOutput, double rightOutput) {
        System.out.println("BaseAutonController: Setting desired voltage= [" + leftOutput+","+ rightOutput+"]");
        db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, leftOutput);
        db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, rightOutput);

    }
    public boolean isFinished() {
        return mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds());
    }
    public void end(boolean interrupted) {
        mTimer.stop();
        if (interrupted) {
            db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, 0);
            db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, 0);
        }
    }
}