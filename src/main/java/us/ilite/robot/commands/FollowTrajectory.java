package us.ilite.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import us.ilite.robot.Robot;
import us.ilite.robot.modules.FalconDriveModule;
import us.ilite.robot.modules.NeoDriveModule;

public class FollowTrajectory implements ICommand {

    private Trajectory mTrajectory;
    private boolean mUsePid;
    private Timer mTrajectoryTimer;
    private RamseteController mController;
    private PIDController mLeftDrivePID;
    private PIDController mRightDrivePID;
    private SimpleMotorFeedforward mFeedForward;
    private Trajectory.State initialState;
    private DifferentialDriveKinematics mDriveKinematics;
    private DifferentialDriveWheelSpeeds mPreviousSpeeds;
    private double mPreviousTime = -1;

    public FollowTrajectory(Trajectory pTrajectory, boolean pUsePid) {
        mTrajectory = pTrajectory;
        mUsePid = pUsePid;
        mController = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedForward = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
        mLeftDrivePID = new PIDController(0.00051968,0,0);
        mRightDrivePID = new PIDController(0.00051968,0,0);
        mTrajectoryTimer = new Timer();
        mDriveKinematics = new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet));
    }
    @Override
    public void init(double pNow) {
        Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
        Robot.DATA.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mTrajectory.getInitialPose().getX());
        Robot.DATA.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mTrajectory.getInitialPose().getY());
        mTrajectoryTimer.reset();
        mTrajectoryTimer.start();
        initialState = mTrajectory.sample(0);
        mPreviousSpeeds =
                mDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        SmartDashboard.putNumber("Initial state X", initialState.poseMeters.getX());
        SmartDashboard.putNumber("Initial state Y", initialState.poseMeters.getY());
        SmartDashboard.putNumber("Trajectory Total Time in Seconds", mTrajectory.getTotalTimeSeconds());
    }

    @Override
    public boolean update(double pNow) {
        double curTime = mTrajectoryTimer.get();
        double dt = curTime - mPreviousTime;
        Trajectory.State setpoint = mTrajectory.sample(mTrajectoryTimer.get());
        ChassisSpeeds chassisSpeeds = mController.calculate(getRobotPose(), setpoint);
        DifferentialDriveWheelSpeeds wheelSpeeds = mDriveKinematics.toWheelSpeeds(chassisSpeeds);
        if (mUsePid) {
            double actualRightSpeed = Units.feet_to_meters(Robot.DATA.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
            double actualLeftSpeed = Units.feet_to_meters(Robot.DATA.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
            double leftFeedforward =
                    mFeedForward.calculate(
                            wheelSpeeds.leftMetersPerSecond, (wheelSpeeds.leftMetersPerSecond - mPreviousSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    mFeedForward.calculate(
                            wheelSpeeds.rightMetersPerSecond, (wheelSpeeds.rightMetersPerSecond - mPreviousSpeeds.rightMetersPerSecond) / dt);

            double leftOutput =
                    leftFeedforward
                            + mLeftDrivePID.calculate(actualLeftSpeed, wheelSpeeds.leftMetersPerSecond);

            double rightOutput =
                    rightFeedforward
                            + mRightDrivePID.calculate(actualRightSpeed, wheelSpeeds.rightMetersPerSecond);
            Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftOutput);
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightOutput);
        }
        else {
            Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, Units.meters_to_feet(wheelSpeeds.leftMetersPerSecond));
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, Units.meters_to_feet(wheelSpeeds.rightMetersPerSecond));
        }
        mPreviousSpeeds = wheelSpeeds;
        mPreviousTime = curTime;
        if (mTrajectoryTimer.get() > mTrajectory.getTotalTimeSeconds()) {
            return true;
        }
        return false;
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
        Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.0);
        Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, 0.0);
    }
    //Helper method to get robot pose
    public Pose2d getRobotPose() {
        double x = Robot.DATA.drivetrain.get(EDriveData.X_ACTUAL_ODOMETRY_METERS);
        double y = Robot.DATA.drivetrain.get(EDriveData.Y_ACTuAL_ODOMETRY_METERS);
        double heading = Robot.DATA.drivetrain.get(EDriveData.ACTUAL_HEADING_RADIANS);
        return new Pose2d(new Translation2d(x, y), new Rotation2d(heading));
    }
}
