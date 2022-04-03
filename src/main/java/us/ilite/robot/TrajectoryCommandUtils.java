package us.ilite.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.robot.modules.NeoDriveModule;
import us.ilite.robot.modules.VioletDriveModule;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryCommandUtils {

    public static Trajectory getJSONTrajectory() {
        String trajectoryJSON = "paths/Unnamed_0.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Unable to open " + trajectoryJSON + " " + Arrays.toString(ex.getStackTrace()));
        }
        return trajectory;
    }

    public static Trajectory getOtherJSONTrajectory() {
        String trajectoryJSON = "paths/second_test_five_ball.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Unable to open " + trajectoryJSON + " " + Arrays.toString(ex.getStackTrace()));
        }
        return trajectory;
    }

    public static Trajectory buildExampleTrajectory() {
        // TODO Normally this method would build the trajectory based off of the waypoints and config that is passed in
        //  but I am going to keep it hard-coded for now
        TrajectoryConfig config = new TrajectoryConfig(2 ,2);
        config.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA),
                new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet)), 10));
        config.addConstraint(new CentripetalAccelerationConstraint(1));
        config.addConstraint(new DifferentialDriveKinematicsConstraint(new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet)), 3));
        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1, 0));
        waypoints.add( new Translation2d(2, 0));
        //TODO figure out how to fix the starting rotation 2d
        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                waypoints,
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
    }

    /**
     * Private constructor to prevent instantiation, since this is a utility class
     */
    private TrajectoryCommandUtils() {

    }


}
