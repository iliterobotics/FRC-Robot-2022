package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

public class ThreeBallTrajectoryController extends BaseAutonController {
    //ROBOT POSES
    public static final Pose2d ROBOT_START = new Pose2d(new Translation2d(28.415, 17.214), Rotation2d.fromDegrees(67.641));
    public static final Pose2d FIRST_BALL = new Pose2d(new Translation2d(29.083, 25.782), Rotation2d.fromDegrees(45.923));
    public static final Pose2d SECOND_BALL = new Pose2d(new Translation2d(37.572, 20.987), Rotation2d.fromDegrees(-70.364));

    //ROBOT TRAJECTORIES
    private Trajectory mFirstBallPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, FIRST_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mSecondBallPath = TrajectoryGenerator.generateTrajectory(List.of(FIRST_BALL, SECOND_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mThreeBallPath = TrajectoryGenerator.generateTrajectory(List.of(SECOND_BALL, ROBOT_START), super.mTrajectoryConfig.setReversed(true));


    //ROBOT COMMANDS
    private FollowTrajectory mFirstTrajectoryCommand = new FollowTrajectory(mFirstBallPath, false);
    private FollowTrajectory mSecondTrajectoryCommand = new FollowTrajectory(mSecondBallPath, false);
    private FollowTrajectory mThirdTrajectoryCommand = new FollowTrajectory(mThreeBallPath, false);

    //ROBOT PATH TIME ENDS
    private double mFirstPathTimeEnd = 3.0;
    private double mSecondPathTimeEnd = mFirstPathTimeEnd + 3.0;
    private double mThirdPathTimeEnd = mSecondPathTimeEnd + 3.0;
    private boolean fireCargo = false;
    private Timer mTimer;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        Robot.FIELD.getObject("First path").setTrajectory(mFirstBallPath);
        Robot.FIELD.getObject("Second path").setTrajectory(mSecondBallPath);
        Robot.FIELD.getObject("Third path").setTrajectory(mThreeBallPath);
        mFirstTrajectoryCommand.init(mTimer.get());
    }
    public void updateImpl() {
        double time = mTimer.get();
        if (fireCargo) {
            fireCargo();
        }
        else {
            intakeCargo();
        }
        if (time < 0.5) {
            fireCargo = true;
        } else if (time < mFirstPathTimeEnd) {
            fireCargo = false;
            mFirstTrajectoryCommand.update(time);
        } else if (time < mFirstPathTimeEnd + 0.1) {
            mSecondTrajectoryCommand.init(time);
        } else if (time < mSecondPathTimeEnd) {
            mSecondTrajectoryCommand.update(time);
        } else if (time < mSecondPathTimeEnd + 0.1) {
            mThirdTrajectoryCommand.init(time);
        } else if (time < mThirdPathTimeEnd) {
            mThirdTrajectoryCommand.update(time);
        }
        else if (time > mThirdPathTimeEnd + 1.5) {
            fireCargo = true;
        }
    }
}
