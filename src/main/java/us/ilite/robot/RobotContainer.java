package us.ilite.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import us.ilite.common.config.Settings;
import us.ilite.robot.modules.DriveModule;
import us.ilite.robot.modules.VioletDriveModule;

import java.util.List;
import java.util.Set;

public class RobotContainer {
    private final VioletDriveModule mDrive;
    public RobotContainer() {
        mDrive = new VioletDriveModule();
    }

    public Command getCommand() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
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

        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        mDrive::getPose,
                        new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                Settings.kS,
                                Settings.kV,
                                Settings.kA),
                        Settings.kDriveKinematics,
                        mDrive::getWheelSpeeds,
                        new PIDController(Settings.kP, 0, 0),
                        new PIDController(Settings.kP, 0, 0),
                        // RamseteCommand passes volts to the callback
                        mDrive::tankDriveVolts, VioletDriveModule.subBase
                        );

        // Reset odometry to the starting pose of the trajectory.
        mDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> mDrive.tankDriveVolts(0, 0));
    }


}
