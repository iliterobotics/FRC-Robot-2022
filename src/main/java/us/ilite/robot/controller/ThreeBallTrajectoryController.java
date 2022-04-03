package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.feet_to_meters;

public class ThreeBallTrajectoryController extends BaseAutonController {
    public static Pose2d
            START_POSITION = new Pose2d(feet_to_meters(25.016), feet_to_meters(5.724), Rotation2d.fromDegrees(270d)),
            SECOND_BALL = new Pose2d(feet_to_meters(25.380), feet_to_meters( 2.733), Rotation2d.fromDegrees(270)),
            SHOT_AGAINST_HUB = new Pose2d(feet_to_meters(25.866), feet_to_meters(10.343), Rotation2d.fromDegrees(-81)),
            THIRD_BALL = new Pose2d(feet_to_meters(16.602), feet_to_meters(6.029), Rotation2d.fromDegrees(-25)),
            INTERMEDIATE_STEP = new Pose2d(feet_to_meters(23.073), feet_to_meters(6.983), Rotation2d.fromDegrees(-60))
    ;

    private final Trajectory
        mFirstLeg = TrajectoryGenerator.generateTrajectory(List.of(START_POSITION,SECOND_BALL),super.mTrajectoryConfig.setReversed(false)),
        mSecondLeg = TrajectoryGenerator.generateTrajectory(List.of(SECOND_BALL,SHOT_AGAINST_HUB),super.mTrajectoryConfig.setReversed(true)),
        mThirdLeg = TrajectoryGenerator.generateTrajectory(List.of(SHOT_AGAINST_HUB,INTERMEDIATE_STEP, THIRD_BALL),super.mTrajectoryConfig.setReversed(false)),
        mFourthLeg = TrajectoryGenerator.generateTrajectory(List.of(THIRD_BALL,INTERMEDIATE_STEP, SHOT_AGAINST_HUB),super.mTrajectoryConfig.setReversed(true))
    ;

    private FollowTrajectory mFollower;
    private Timer mTimer = new Timer();

    public void initialize() {
        mTimer.reset();
        mTimer.start();

//        Robot.FIELD.getObject("FirstLeg").setTrajectory(mFirstLeg);
//        Robot.FIELD.getObject("First Shoot").setTrajectory(mFirstShoot);
//        Robot.FIELD.getObject("Second Leg Ball 3").setTrajectory(mSecondLegBall3);
//        Robot.FIELD.getObject("Second Leg Ball 4").setTrajectory(mSecondLegBall4);
//        Robot.FIELD.getObject("Final Shot").setTrajectory(mFinalShot);

        mFollower = new FollowTrajectory(mFirstLeg, false);
        mFollower.init(mTimer.get());
    }

    public void updateImpl() {
//        double time = mTimer.get();
//        intakeCargo();
//        mFollower.update(time);
//
//        mFirstLeg.getTotalTimeSeconds();
    }
}
