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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import us.ilite.common.config.Settings;
import us.ilite.robot.modules.VioletDriveModule;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

public class TrajectoryCommandUtils {


    public static Trajectory getJSONTrajectory() {
        String trajectoryJSON = "paths/test.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Successful");
        } catch (IOException ex) {
            System.out.println("Unable to open " + trajectoryJSON + " " + Arrays.toString(ex.getStackTrace()));
        }
        System.out.println("DONE");
        return trajectory;
    }

    /**
     * Private constructor to prevent instantiation, since this is a utility class
     */
    private TrajectoryCommandUtils() {

    }


}
