package us.ilite.robot.controller;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.modules.FalconDriveModule;
import us.ilite.robot.modules.NeoDriveModule;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import static us.ilite.common.lib.util.Units.feet_to_meters;

/**
 * Class responsible for executing {@link Trajectory} and moving the
 * robot along said Trajectory. This is an autonmous routine that can
 * accepty and Trajectory, so long as it is feasible (mathematically and
 * realistically).
 */
public class BaseAutonController extends AbstractController {

    protected DifferentialDriveKinematics k = new DifferentialDriveKinematics(feet_to_meters(NeoDriveModule.kTrackWidthFeet));
    protected final TrajectoryConfig mTrajectoryConfig = new TrajectoryConfig(1.0, 1.0).
            addConstraint( new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA), k, 12)).
            addConstraint(new CentripetalAccelerationConstraint(1.0)).
            addConstraint(new DifferentialDriveKinematicsConstraint(k, 1.0)).
            setStartVelocity(0.0).
            setEndVelocity(0.0);



    /**
     * Default constructor. This will instantiate the variables that are not dependent on the init
     * state of autonomous. For those components, those will be initialized in the initialized method
     */
    public BaseAutonController() {

    }

    public void initialize() {
        initialize(null);
    }

    /**
     * Method that should be called when the autonomous state is initialized. This is so that the overall
     * state can be restarted, should the autonomous be run multiple times. (Something that only really happens
     * at home).
     */
    public void initialize(Trajectory pTrajectory) {

    }
    @Override
    protected void updateImpl() {

    }

    public Pose2d getStartPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }

}