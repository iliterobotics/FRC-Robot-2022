package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.feet_to_meters;

public class ThreeBallTrajectoryController extends BaseAutonController {
    //ROBOT POSES
    public static final Pose2d ROBOT_START = new Pose2d(new Translation2d(Units.feet_to_meters(28.415), Units.feet_to_meters(17.214)), Rotation2d.fromDegrees(67.641));
    public static final Pose2d FIRST_BALL = new Pose2d(new Translation2d(Units.feet_to_meters(34.858), Units.feet_to_meters(20.018)), Rotation2d.fromDegrees(0d));
    public static final Pose2d SECOND_BALL = new Pose2d(new Translation2d(Units.feet_to_meters(46.532), Units.feet_to_meters(21.699)), Rotation2d.fromDegrees(40.681));
    //Fix these waypoints to ensure that we have an ACTUAL feasible path to follow on thet way back...
    public static final Translation2d RETURN_WAYPOINT = new Translation2d(Units.feet_to_meters(40.549), Units.feet_to_meters(19.409));
    public static final Translation2d RETURN_WAYPOINT_TWO = new Translation2d(Units.feet_to_meters(31.044), Units.feet_to_meters(19.222));


    //ROBOT TRAJECTORIES
    //About 3.658323 seconds for both
    private Trajectory mFirstBallPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, FIRST_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mFirstReturnPath = TrajectoryGenerator.generateTrajectory(List.of(FIRST_BALL, ROBOT_START), super.mTrajectoryConfig.setReversed(true));
    private Trajectory mHumanStationPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, SECOND_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mHumanStationReturnPath = TrajectoryGenerator.generateTrajectory(ROBOT_START, List.of(RETURN_WAYPOINT, RETURN_WAYPOINT_TWO), SECOND_BALL, super.mTrajectoryConfig.setReversed(true));

    //   private Trajectory mSecondBallPath = TrajectoryGenerator.generateTrajectory()
    private Trajectory mActiveTrajectory;

    //command that will actually drive the robot
    private FollowTrajectory mFollower;

    //ROBOT PATH TIME ENDS
    private double mShotTime = 0.5;
    private double m2ndBallLegTime = 0;
    private double mReturnShootLeg = 0;
    private double mSecondShotTime = 0;
    private double mHumanStationPathTime = 0;
    private double mHumanStationReturnPathTime = 0;

    private Timer mTimer;
    private boolean fire = false;
    private boolean mFirstPathDone;
    private boolean mFirstReturnPathDone;
    private boolean mHumanStationPathDone;
    private boolean mHumanStationReturnPathDone;

    public void initialize() {
        SmartDashboard.putNumber("Human Station Path Time", mHumanStationPath.getTotalTimeSeconds());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        m2ndBallLegTime = mShotTime + mFirstBallPath.getTotalTimeSeconds();
        mReturnShootLeg = m2ndBallLegTime + mFirstReturnPath.getTotalTimeSeconds();
        mSecondShotTime = mReturnShootLeg + 0.5;
        mHumanStationPathTime = mSecondShotTime + mHumanStationPath.getTotalTimeSeconds();
        mHumanStationReturnPathTime = mHumanStationPathTime + mHumanStationReturnPath.getTotalTimeSeconds();
        mFollower = new FollowTrajectory(mFirstBallPath, false);
        mFollower.init(mTimer.get());
        mActiveTrajectory = new Trajectory();
    }
    public void updateImpl() {
        double time = mTimer.get();
        Robot.FIELD.getObject("Active Path").setTrajectory(mActiveTrajectory);
        if (fire) {
            fireCargo();
        } else {
            intakeCargo();
        }
        if (time < mShotTime) {
            mActiveTrajectory = mFirstReturnPath;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
            db.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getX());
            db.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getY());
            fire = true;
        }
        else if (time < m2ndBallLegTime) { // Move to Ball #1 (in the direction of human player station)
            fire = false;
            mFollower.update(time);
        }
        else if (time < m2ndBallLegTime + 0.05) {
            mActiveTrajectory = mFirstReturnPath;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
            db.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getX());
            db.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getY());
            mFollower = new FollowTrajectory(mActiveTrajectory, false);
            mFollower.init(time);
        } else if (time < mReturnShootLeg) { //Move back to initial starting location
            mFollower.update(time);
        }  else if (time < mSecondShotTime) {
            mActiveTrajectory = mHumanStationPath;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
            db.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getX());
            db.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getY());
            mFollower = new FollowTrajectory(mActiveTrajectory, false);
            mFollower.init(time);
            fire = true;
        } else if (time < mHumanStationPathTime) {
            mFollower.update(time);
        } else if (time < mHumanStationPathTime + 0.05) {
            mActiveTrajectory = mHumanStationReturnPath;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
            db.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getX());
            db.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mActiveTrajectory.getInitialPose().getY());
            mFollower = new FollowTrajectory(mActiveTrajectory, false);
            mFollower.init(time);
        } else if (time < mHumanStationReturnPathTime) {
            mFollower.update(time);
        } else {
            mFollower = null;
            stopDrivetrain();
            fire = true;
        }
        Robot.FIELD.setRobotPose(getRobotPose());
    }

    public Pose2d getStartPose() {
        return ROBOT_START;
    }
    public Pose2d getRobotPose() {
        double x = Robot.DATA.drivetrain.get(EDriveData.X_ACTUAL_ODOMETRY_METERS);
        double y = Robot.DATA.drivetrain.get(EDriveData.Y_ACTuAL_ODOMETRY_METERS);
        double heading = Robot.DATA.drivetrain.get(EDriveData.ACTUAL_HEADING_RADIANS);
        return new Pose2d(new Translation2d(x, y), new Rotation2d(heading));
    }
}
