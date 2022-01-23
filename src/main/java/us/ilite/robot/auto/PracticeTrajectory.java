package us.ilite.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import us.ilite.common.config.Settings;
import us.ilite.robot.modules.VioletDriveModule;

public class PracticeTrajectory {
    //This is the 2018 code copied from the WPILib Documentation
    //Never use this method
    public static Trajectory generateTrajectory() {

        // 2018 cross scale auto waypoints.
        Pose2d sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
                Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
                Rotation2d.fromDegrees(-160));

        ArrayList <Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }

    public static Trajectory runStraight() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Settings.kS,
                                Settings.kV,
                                Settings.kA),
                        Settings.kDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        VioletDriveModule.kMaxVelocityMS,
                        VioletDriveModule.kMaxVelocityMS)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Settings.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints
                List.of(new Translation2d(0, 0.5), new Translation2d(0, .75)),
                //End Somewhere
                new Pose2d(0, 2, new Rotation2d((Math.PI)/2)),
                // Pass config
                config);
    }
}