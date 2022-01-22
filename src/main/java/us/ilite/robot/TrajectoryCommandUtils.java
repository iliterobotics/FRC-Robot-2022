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
import us.ilite.common.config.Settings;
import us.ilite.robot.modules.VioletDriveModule;

import java.util.List;

public class TrajectoryCommandUtils {

    /**
     * Method to build the necessary {@link Command} for running a trajectory. This
     * method will run a basic Trajectory and is designed for testing
     * @param pDriveModule
     *  The drive module that is used by the Trajectory for actually moving the robot
     * @return
     *  A fully constructed command that can be used to move the robot through a trajectory
     */
    public static Command buildTrajectoryCommand(VioletDriveModule pDriveModule) {
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

        return buildTrajectoryCommand(pDriveModule, exampleTrajectory);
    }

    /**
     * Method to build the necessary {@link Command} for running a trajectory. This method will accept a Trajectory and
     * is designed.
     * @param pDriveModule
     *  The drive module that is used to control the drive train motors
     * @param pExampleTrajectory
     *  The trajectory to use
     * @return
     *  A command that can be used to move the robot along the desired trajectory
     */
    public static Command buildTrajectoryCommand(VioletDriveModule pDriveModule, Trajectory pExampleTrajectory) {
        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        pExampleTrajectory,
                        pDriveModule::getPose,
                        new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                Settings.kS,
                                Settings.kV,
                                Settings.kA),
                        Settings.kDriveKinematics,
                        pDriveModule::getWheelSpeeds,
                        new PIDController(Settings.kP, 0, 0),
                        new PIDController(Settings.kP, 0, 0),
                        // RamseteCommand passes volts to the callback
                        pDriveModule::tankDriveVolts, VioletDriveModule.subBase
                );

        // Reset odometry to the starting pose of the trajectory.
        pDriveModule.resetOdometry(pExampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> pDriveModule.tankDriveVolts(0, 0));

    }

    /**
     * Private constructor to prevent instantiation, since this is a utility class
     */
    private TrajectoryCommandUtils() {

    }


}
