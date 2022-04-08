package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.lib.util.Units;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.feet_to_meters;

public class ThreeBallTrajectoryController extends BaseAutonController {
    //ROBOT POSES
    public static final Pose2d ROBOT_START = new Pose2d(new Translation2d(Units.feet_to_meters(28.415), Units.feet_to_meters(17.214)), Rotation2d.fromDegrees(67.641));
    public static final Pose2d FIRST_BALL = new Pose2d(new Translation2d(Units.feet_to_meters(29.083), Units.feet_to_meters(24.782)), Rotation2d.fromDegrees(90d));
    public static final Pose2d SECOND_BALL = new Pose2d(new Translation2d(Units.feet_to_meters(36.572), Units.feet_to_meters(20.987)), Rotation2d.fromDegrees(0d));

    //ROBOT TRAJECTORIES
    private Trajectory mFirstBallPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, FIRST_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mSecondBallPath = TrajectoryGenerator.generateTrajectory(List.of(FIRST_BALL, ROBOT_START), super.mTrajectoryConfig.setReversed(true));
    private Trajectory mThreeBallPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, SECOND_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mThreeBallReturnPath = TrajectoryGenerator.generateTrajectory(List.of(SECOND_BALL, ROBOT_START), super.mTrajectoryConfig.setReversed(true));

    private FollowTrajectory mFollower;

    //ROBOT COMMANDS
    private FollowTrajectory mFirstTrajectoryCommand = new FollowTrajectory(mFirstBallPath, false);
    private FollowTrajectory mSecondTrajectoryCommand = new FollowTrajectory(mSecondBallPath, false);
    private FollowTrajectory mThirdTrajectoryCommand = new FollowTrajectory(mThreeBallPath, false);
    private FollowTrajectory mFourthTrajectoryCommand = new FollowTrajectory(mThreeBallReturnPath, false);

    //ROBOT PATH TIME ENDS
    private double mFirstPathTimeEnd = 3.0;
    private double mSecondPathTimeEnd = mFirstPathTimeEnd + 3.0;
    private double mFirstShootTimeEnd = mSecondPathTimeEnd + 0.5;
    private double mThirdPathTimeEnd = mFirstShootTimeEnd + 3.0;
    private double mFourthPathTimeEnd = mThirdPathTimeEnd + 3.0;
    private double mSecondShootTimeEnd = mSecondPathTimeEnd + 0.5;

    private Timer mTimer;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();

        Robot.FIELD.getObject("First path").setTrajectory(mFirstBallPath);
        Robot.FIELD.getObject("Second path").setTrajectory(mSecondBallPath);
        Robot.FIELD.getObject("Third path").setTrajectory(mThreeBallPath);
        Robot.FIELD.getObject("Fourth path").setTrajectory(mThreeBallReturnPath);

        mFollower = new FollowTrajectory(mFirstBallPath, false);
        mFollower.init(mTimer.get());
    }
    public void updateImpl() {
        double time = mTimer.get();
        if (time < mFirstPathTimeEnd) { // Move to Ball #2
            mFirstTrajectoryCommand.update(time);
            intakeCargo();
        } else if (time == mFirstPathTimeEnd) {
            mFollower = mSecondTrajectoryCommand;
            mFollower.init(time);
        } else if (time < mSecondPathTimeEnd) { // Move Back to Hub
            mFollower.update(time);
            stageBalls();
        } else if (time < mFirstShootTimeEnd) { // Shoot Ball #1 and Ball #2
            fireCargo();
        } else if (time == mFirstShootTimeEnd) {
            mFollower = mThirdTrajectoryCommand;
            mFollower.init(time);
        } else if (time < mThirdPathTimeEnd) { // Move Towards Ball #3
            mFollower.update(time);
            intakeCargo();
        } else if (time == mThirdPathTimeEnd) {
            mFollower = mFourthTrajectoryCommand;
            mFollower.init(time);
        } else if (time < mFourthPathTimeEnd) { // Move Back to Hub
            mFollower.update(time);
            stageBalls();
        } else { // Shoot Ball #3
            fireCargo();
        }
    }

    public Pose2d getStartPose() {
        return ROBOT_START;
    }
}
