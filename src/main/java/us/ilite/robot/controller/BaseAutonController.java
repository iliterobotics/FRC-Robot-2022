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

    /**
     * Timer used to get clock time in seconds. This is the time off of the
     * robot and usually doesn't represent real time. This is only intended to be
     * used for temporal offsets.
     */
    public final Timer mTimer;
    public final Timer mFirstLeg;
    /**
     * The {@link RamseteController} that is used to follow a {@link Trajectory}. This
     * will be used to calculate the chasis speed based on the robots pose and velocity
     * at a given time. It will also look up where the robot should be along the trajectory
     */
    public final RamseteController mFollower;
    /**
     * The module responsible for calculating how much volatage to send to the motors based on the
     * desired velocity.
     */
    public final SimpleMotorFeedforward mFeedforward;
    /**
     * The kinematics of the motors based on the distance between the wheels (left and right).
     */
    public final DifferentialDriveKinematics mDriveKinematics;

    public final PIDController mMotorPidController;
    /**
     * The trajectory to execute. At this time this class reaches out and gets the trajectory. Since
     * the trajectory needs to be restarted every time, this is not final and is reloaded in the init method.
     */
    public Trajectory mTrajectory;
    /**
     * A history of the speeds of the wheels this module has calculated for the trajectory
     */
    private DifferentialDriveWheelSpeeds mPrevTargetWheelSpeeds;
    /**
     * A history of the speeds of the actual speeds the robot moved
     */
    public DifferentialDriveWheelSpeeds mPrevActualSpeed;
    /**
     * The time, in seconds. This may not reflect realtime.
     */
    public double mPrevTime;

    /**
     * Unique identifier for this object. This should get reinitialized on each initialize call
     *
     */
    private UUID mID;
    public Trajectory.State initialState;
    public int mCycleCount = 0;

    protected final TrajectoryConfig mTrajectoryConfig;

    /**
     * Default constructor. This will instantiate the variables that are not dependent on the init
     * state of autonomous. For those components, those will be initialized in the initialized method
     */
    public BaseAutonController() {
        mFollower = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
        mMotorPidController = new PIDController(0.00051968,0,0);
        mTimer = new Timer();
        mFirstLeg = new Timer();
        mDriveKinematics = new DifferentialDriveKinematics(Units.feet_to_meters(FalconDriveModule.kTrackWidthFeet));
        SmartDashboard.putNumber("trajectory-seconds",-1);


        mTrajectoryConfig = new TrajectoryConfig(1.0, 1.0);
        DifferentialDriveKinematics k = new DifferentialDriveKinematics(feet_to_meters(NeoDriveModule.kTrackWidthFeet));
        mTrajectoryConfig.addConstraint(
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA),
                        k,
                        10
                )
        );
        mTrajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(1.0));
        mTrajectoryConfig.addConstraint(new DifferentialDriveKinematicsConstraint(k,1.0));
        mTrajectoryConfig.setStartVelocity(0.0);
        mTrajectoryConfig.setEndVelocity(0.0);
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
        mID = UUID.randomUUID();
        mTimer.reset();
        mTimer.start();
        mFirstLeg.reset();
        mFirstLeg.start();
        startTime = mTimer.get();
        mMotorPidController.reset();
        mPrevTime = -1;
        mTrajectory = pTrajectory;
        if(mTrajectory != null) {
            initialState = mTrajectory.sample(0);
            SmartDashboard.putNumber("Initial state x", initialState.poseMeters.getX());
            SmartDashboard.putNumber("Initial state y", initialState.poseMeters.getY());
            SmartDashboard.putNumber("Trajectory Total Time in Seconds", mTrajectory.getTotalTimeSeconds());
        }
        mPrevTargetWheelSpeeds = new DifferentialDriveWheelSpeeds(0,0);
        mPrevActualSpeed = new DifferentialDriveWheelSpeeds(0,0);
    }
    @Override
    protected void updateImpl() {

    }

    public Pose2d getStartPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    private static int EXEC_COUNT = 1;
    private static boolean HAS_FINISHED = false;
    private double startTime = 0;

    /**
     * Method to perform the actual traversal. This should run a single step. A step will be defined as
     * a sequence of execution between some delta time from the previous execution. This method is expected to be
     * called multiple times until the robot traverses the entire Trajectory or until autonmous runs out of time.
     */
    public void execute() {
        double curTime = mTimer.get() - startTime;
        double dT = curTime - mPrevTime;


        if (mPrevTime < 0) {
            updateDriveTrain(new ImmutablePair<Double,Double>(0d,0d));
            mPrevTime = curTime;
            return;
        }

        Pose2d robotPose = getRobotPose();
        Trajectory.State sample = mTrajectory.sample(curTime);

        List<Object>data = new ArrayList<>();
        data.add(curTime);
        data.add(getRobotPose().getX());
        data.add(Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT)));
        data.add(Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT)));
        data.add(sample.velocityMetersPerSecond);
        data.add(sample.accelerationMetersPerSecondSq);


        DifferentialDriveWheelSpeeds targetWheelSpeeds = getTargetWheelSpeeds(sample, robotPose, data);
        DifferentialDriveWheelSpeeds actualSpeeds = calculateActualSpeeds();
        logDataToSmartDashboard(dt, sample, targetWheelSpeeds, actualSpeeds);

        data.add(actualSpeeds.leftMetersPerSecond);
        data.add(actualSpeeds.rightMetersPerSecond);

        double instActualAccelLeft = (actualSpeeds.leftMetersPerSecond - mPrevActualSpeed.leftMetersPerSecond) / dT;
        double instActualAccelRight = (actualSpeeds.rightMetersPerSecond - mPrevActualSpeed.rightMetersPerSecond) / dT;

        mPrevActualSpeed = new DifferentialDriveWheelSpeeds(actualSpeeds.leftMetersPerSecond, actualSpeeds.rightMetersPerSecond);

        data.add(instActualAccelLeft);
        data.add(instActualAccelRight);

        ramseteFollow(curTime, dT, actualSpeeds, targetWheelSpeeds, data);
    }

    protected void ramseteFollow(double curTime, double dT, DifferentialDriveWheelSpeeds actualSpeeds,
                                 DifferentialDriveWheelSpeeds targetWheelSpeeds, List<Object>data) {
        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
        MutablePair<Double,Double> output = new MutablePair<>();

        double leftSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward =
                calculateFeedsForward(leftSetpoint, mPrevTargetWheelSpeeds.leftMetersPerSecond, dT);
        double rightFeedforward =
                calculateFeedsForward(rightSetpoint, mPrevTargetWheelSpeeds.rightMetersPerSecond, dT);

        output.left = calculateOutputFromFeedForward(leftFeedforward, mMotorPidController, actualSpeeds.leftMetersPerSecond, targetWheelSpeeds.leftMetersPerSecond,data);
        output.right = calculateOutputFromFeedForward(rightFeedforward, mMotorPidController, actualSpeeds.rightMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond,data);
        if(data != null) {
            data.add(leftSetpoint);
            data.add(rightSetpoint);
            data.add(leftFeedforward);
            data.add(rightFeedforward);
            data.add(mPrevTargetWheelSpeeds.leftMetersPerSecond);
            data.add(mPrevTargetWheelSpeeds.rightMetersPerSecond);
            data.add(output.left);
            data.add(output.right);
            CSVToLogFile.getInstance().logCSVData(data, this.getClass());
        }
        updateDriveTrain(output);
        mPrevTargetWheelSpeeds = targetWheelSpeeds;
        mPrevTime = curTime;
    }

    /**
     * Method to calculate the feed forward step of the trajectory traversal
     * @param pSpeedSetpoint The speed set point
     * @param pPrevSpeedsMetersPerSec The previous speed in meters/sec
     * @param pDt The amount of time that has passed since the last execution
     * @return The feeds forward from the gains and setpoint
     */
    private double calculateFeedsForward(double pSpeedSetpoint, double pPrevSpeedsMetersPerSec, double pDt) {
        return mFeedforward.calculate(
                pSpeedSetpoint, (pSpeedSetpoint - pPrevSpeedsMetersPerSec) / pDt);
    }

    /**
     * Method to calculate the feed foward plus the pid output from actual speed and setpoint
     * @param pFeedForward
     *  The feed forwards
     * @param pPidController
     *  the PID controller
     * @param pActualSpeeds
     *  The actual speeds (m/s)
     * @param pSetPoint
     *  The set point
     * @return
     *  The feed forward + the output from the pid controller given the actual speed and setpoints
     */
    private double calculateOutputFromFeedForward(double pFeedForward, PIDController pPidController, double pActualSpeeds, double pSetPoint, List<Object>data) {
        double pidCalc = pPidController.calculate(pActualSpeeds, pSetPoint);
        if(data != null) {
            data.add(pidCalc);
        }

        return pFeedForward + pidCalc;
    }

    /**
     * Method to calcualte the actual speeds
     * @return
     *  Return the {@link DifferentialDriveWheelSpeeds}
     */
    private DifferentialDriveWheelSpeeds calculateActualSpeeds() {
        double actualLeftSpeedMeters = Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
        double actualRightSpeedMeters = Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s));
        return new DifferentialDriveWheelSpeeds(actualLeftSpeedMeters, actualRightSpeedMeters);
    }

    /**
     * Method to get the target wheel speeds
     * @param trajectorySample
     *  The sample to use in the calculation
     * @param pRobotPose
     *  The current robot pose
     * @return
     *  The robot wheel speeds
     */
    private DifferentialDriveWheelSpeeds getTargetWheelSpeeds(Trajectory.State trajectorySample, Pose2d pRobotPose, List<Object>data) {
        ChassisSpeeds calculate = mFollower.calculate(pRobotPose, trajectorySample);
        data.add(calculate.vxMetersPerSecond);
        data.add(calculate.vyMetersPerSecond);
        data.add(calculate.omegaRadiansPerSecond);

        DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds = mDriveKinematics.toWheelSpeeds(calculate);
        data.add(differentialDriveWheelSpeeds.leftMetersPerSecond);
        data.add(differentialDriveWheelSpeeds.rightMetersPerSecond);
        return differentialDriveWheelSpeeds;
    }


    /**
     * Get the current robot pose
     * @return
     *  The robot pose represented as {@link Pose2d}
     */
    private Pose2d getRobotPose() {
        double absX = initialState.poseMeters.getX() + db.drivetrain.get(EDriveData.GET_X_OFFSET_METERS);
        double absY = initialState.poseMeters.getY() + db.drivetrain.get(EDriveData.GET_Y_OFFSET_METERS);
        Rotation2d r2d = new Rotation2d(db.drivetrain.get(EDriveData.ACTUAL_HEADING_RADIANS));
        Pose2d robotPose = new Pose2d(new Translation2d(absX, absY), r2d);
        return robotPose;
    }

    /**
     * Method to update the motors
     * @param pOutput
     *  The voltage values to set the motors
     */
    private void updateDriveTrain(Pair<Double,Double> pOutput) {
        double leftFeet = Units.meters_to_feet(pOutput.getLeft());
        double rightFeet = Units.meters_to_feet(pOutput.getRight());

        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftFeet);
        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightFeet);
    }

    /**
     * Method to check if the trajectory is finished
     * @return
     *  true if the trajectory traversal is complete, false otherwise
     */
    public boolean isFinished() {
        return mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds());
    }


    private void logDataToSmartDashboard(double dt,Trajectory.State sample, DifferentialDriveWheelSpeeds targetWheelSpeeds, DifferentialDriveWheelSpeeds actualSpeeds) {
        SmartDashboard.putNumber("Exec Count", EXEC_COUNT++);
        boolean finished = isFinished();
        if(finished && !HAS_FINISHED) {
            HAS_FINISHED = true;
            SmartDashboard.putNumber("Trajectory finished time", mTimer.get());
            Pose2d finalPose = getRobotPose();
            SmartDashboard.putNumber("Final Pose X: ", finalPose.getX());
            SmartDashboard.putNumber("Final Pose Y: ", finalPose.getY());
            SmartDashboard.putNumber("Final Pose heading: ", finalPose.getRotation().getDegrees());
        }
        SmartDashboard.putBoolean("Trajectory is Finished", finished);
        SmartDashboard.putNumber("Delta Time",dt);
        SmartDashboard.putNumber("TrajectoryX", sample.poseMeters.getX());
        SmartDashboard.putNumber("TrajectoryY", sample.poseMeters.getY());
        SmartDashboard.putNumber("TrajectoryDegrees", sample.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("TrajectoryVelocity", sample.velocityMetersPerSecond);
        SmartDashboard.putNumber("TrajectoryAccel", sample.accelerationMetersPerSecondSq);
        SmartDashboard.putNumber("Target Speed Left: ", targetWheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Target Speed Right: ", targetWheelSpeeds.rightMetersPerSecond);
        SmartDashboard.putNumber("Actual Speed Left: ", actualSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Actual Speed Right", actualSpeeds.rightMetersPerSecond);
    }
}