package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.*;

public class FourBallTrajectoryAuton extends BaseAutonController{

    private final Trajectory mFirstLeg = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START,FIRST_BALL),super.mTrajectoryConfig.setReversed(false));
    private final Trajectory mFirstShoot = TrajectoryGenerator.generateTrajectory(List.of(FIRST_BALL,SHOOTING),super.mTrajectoryConfig.setReversed(true));
    private final Trajectory mSecondLegBall3 = TrajectoryGenerator.generateTrajectory(List.of(SHOOTING,SECOND_BALL),super.mTrajectoryConfig.setReversed(false));
    private final Trajectory mSecondLegBall4 = TrajectoryGenerator.generateTrajectory(List.of(SECOND_BALL,THIRD_BALL),super.mTrajectoryConfig.setReversed(false));
    private final Trajectory mFinalShot = TrajectoryGenerator.generateTrajectory(List.of(THIRD_BALL,SHOOTING),super.mTrajectoryConfig.setReversed(true));

    private FollowTrajectory mFollower;
    private Timer mTimer = new Timer();

    public void initialize() {


        mTimer.reset();
        mTimer.start();



        Robot.FIELD.getObject("First Leg").setTrajectory(mFirstLeg);
        Robot.FIELD.getObject("First Shoot").setTrajectory(mFirstShoot);
        Robot.FIELD.getObject("Second Leg Ball 3").setTrajectory(mSecondLegBall3);
        Robot.FIELD.getObject("Second Leg Ball 4").setTrajectory(mSecondLegBall4);
        Robot.FIELD.getObject("Final Shot").setTrajectory(mFinalShot);

        mFollower = new FollowTrajectory(mFirstLeg, false);
        mFollower.init(mTimer.get());
    }

    public void updateImpl() {

        double time = mTimer.get();
        intakeCargo();
        mFollower.update(time);

        mFirstLeg.getTotalTimeSeconds();

    }


    public static final Pose2d
        ROBOT_START = new Pose2d(feet_to_meters(25.016), feet_to_meters(5.724), Rotation2d.fromDegrees(270d)),
        FIRST_BALL = new Pose2d(feet_to_meters(25.380), feet_to_meters( 2.733), Rotation2d.fromDegrees(270)),
        SHOOTING = new Pose2d(feet_to_meters(25.349), feet_to_meters( 9.639), Rotation2d.fromDegrees(248)),
        SECOND_BALL = new Pose2d(feet_to_meters(18.444), feet_to_meters(5.732), Rotation2d.fromDegrees(180)),
        THIRD_BALL = new Pose2d(feet_to_meters(5.361), feet_to_meters(4.581), Rotation2d.fromDegrees(225))
    ;

    @Override
    public Pose2d getStartPose() {
        return ROBOT_START;
    }
}
