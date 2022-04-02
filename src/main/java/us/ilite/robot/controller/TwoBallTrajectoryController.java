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
import us.ilite.common.lib.util.Units;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.commands.FollowTrajectory;
import us.ilite.robot.modules.NeoDriveModule;

import java.util.ArrayList;

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

    public static Trajectory buildExampleTrajectory() {
        // TODO Normally this method would build the trajectory based off of the waypoints and config that is passed in
        //  but I am going to keep it hard-coded for now
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        config.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA),
                new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet)), 12));
        config.addConstraint(new CentripetalAccelerationConstraint(0.5));
        config.addConstraint(new DifferentialDriveKinematicsConstraint(new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet)), 3));
        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(new Translation2d(Units.inches_to_meters(12), Units.inches_to_meters(0)), new Rotation2d(0)));
        waypoints.add(new Pose2d(new Translation2d(Units.inches_to_meters(36), Units.inches_to_meters(8)), new Rotation2d(14)));
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }
}

