package us.ilite.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private static class TrajectoryWrapper extends Trajectory {
        private Trajectory mTraj;
        TrajectoryWrapper(Trajectory pTraj) {
            mTraj = pTraj;
        }

        @Override
        public List<State> getStates() {
            return mTraj.getStates();
        }

        @Override
        public Pose2d getInitialPose() {
            return mTraj.getInitialPose();
        }

        @Override
        public double getTotalTimeSeconds() {
            return 4;
        }

        @Override
        public State sample(double timeSeconds) {
            return mTraj.sample(timeSeconds);
        }

        @Override
        public Trajectory transformBy(Transform2d transform) {
            return transformBy(transform);
        }

        @Override
        public Trajectory relativeTo(Pose2d pose) {
            return mTraj.relativeTo(pose);
        }

        @Override
        public Trajectory concatenate(Trajectory other) {
            return mTraj.concatenate(other);
        }

        @Override
        public String toString() {
            return mTraj.toString();
        }

        public int hashCode() {
            return mTraj.hashCode();
        }
        public boolean equals(Object other) {
            return mTraj.equals(other);
        }

    }


    public static Trajectory getJSONTrajectory() {
        String trajectoryJSON = "paths/StraightPush.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = new TrajectoryWrapper(TrajectoryUtil.fromPathweaverJson(trajectoryPath));
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
