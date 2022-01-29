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
    private final Trajectory mTrajectory;

    private Pose2d mPoses;
    private final RamseteController mFollower;
    private final SimpleMotorFeedforward mFeedforward;
    private DifferentialDriveKinematics mDriveKinematics;
    private DifferentialDriveWheelSpeeds mSpeeds;
    private final PIDController mLeftController;
    private final PIDController mRightController;
    private DifferentialDriveWheelSpeeds mPrevSpeeds;
    private double mPrevTime;

    public BaseAutonController() {
        setEnabled(true);
        mTimer = new Timer();
        mPrevTime = -1;
        mDriveKinematics = new DifferentialDriveKinematics(Settings.kTrackwidthMeters);
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
        mUsePid = true;
        mFollower = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kP, Settings.kA);
        mRightController = new PIDController(Settings.kP, 0, 0);
        mLeftController = new PIDController(Settings.kP, 0, 0);
        mLeftController.reset();
        mRightController.reset();
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
            db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, 0.0);
            db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, 0.0);
            mPrevTime = curTime;
            return;
        }
        Rotation2d r2d = new Rotation2d(Units.degrees_to_radians(db.drivetrain.get(EDriveData.DELTA_HEADING)));
        mPoses = new Pose2d(db.drivetrain.get(EDriveData.GET_X_OFFSET), db.drivetrain.get(EDriveData.GET_Y_OFFSET), r2d);

        DifferentialDriveWheelSpeeds targetDriveSpeeds = mDriveKinematics.toWheelSpeeds(mFollower.calculate(mPoses, mTrajectory.sample(curTime)));
        double leftSpeedSetpoint = targetDriveSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetDriveSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        mSpeeds = new DifferentialDriveWheelSpeeds(Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)
        ), Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s)));

        if (mUsePid) {
            double leftFeedforward =
                    mFeedforward.calculate(
                            leftSpeedSetpoint, (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    mFeedforward.calculate(
                            rightSpeedSetpoint, (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt);

            leftOutput =
                    leftFeedforward
                            + mLeftController.calculate(mSpeeds.leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput =
                    rightFeedforward
                            + mRightController.calculate(mSpeeds.rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }
        db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, leftOutput);
        db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, rightOutput);
        mPrevSpeeds = targetDriveSpeeds;
        mPrevTime = curTime;
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