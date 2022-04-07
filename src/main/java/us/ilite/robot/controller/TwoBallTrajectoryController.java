package us.ilite.robot.controller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.config.Settings;
import static us.ilite.common.lib.util.Units.*;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.commands.FollowTrajectory;
import us.ilite.robot.modules.NeoDriveModule;

import java.util.ArrayList;
import java.util.List;

public class TwoBallTrajectoryController extends BaseAutonController {
    private Timer mTimer;
    private FollowTrajectory mFirstTrajectory;

    public void initialize() {
        //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mFirstTrajectory = new FollowTrajectory(buildExampleTrajectory(), false);
        mFirstTrajectory.init(mTimer.get());
    }

    public void updateImpl() {
        double time = mTimer.get();
      //  intakeCargo();
        mFirstTrajectory.update(time);
    }

    public Trajectory buildExampleTrajectory() {
        // TODO Normally this method would build the trajectory based off of the waypoints and config that is passed in
        //  but I am going to keep it hard-coded for now
//        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
//        waypoints.add(new Pose2d(new Translation2d(inches_to_meters(10), inches_to_meters(0)), new Rotation2d(0)));
//        waypoints.add(new Pose2d(new Translation2d(inches_to_meters(36), inches_to_meters(8)), new Rotation2d(14)));
//        return TrajectoryGenerator.generateTrajectory(waypoints, config);
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0d)),
                List.of(
                        new Translation2d(inches_to_meters(6), Rotation2d.fromDegrees(1)),
                        new Translation2d(inches_to_meters(12), Rotation2d.fromDegrees(3))
                ),
                new Pose2d(inches_to_meters(36), inches_to_meters(12), Rotation2d.fromDegrees(14)),
                super.mTrajectoryConfig
        );
    }
}

