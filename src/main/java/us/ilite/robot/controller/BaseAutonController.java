package us.ilite.robot.controller;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.json.JSONObject;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.modules.VioletDriveModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;

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
    private final Timer mTimer;
    /**
     * The {@link RamseteController} that is used to follow a {@link Trajectory}. This
     * will be used to calculate the chasis speed based on the robots pose and velocity
     * at a given time. It will also look up where the robot should be along the trajectory
     */
    private final RamseteController mFollower;
    /**
     * The module responsible for calculating how much volatage to send to the motors based on the
     * desired velocity.
     */
    private final SimpleMotorFeedforward mFeedforward;
    /**
     * The kinematics of the motors based on the distance between the wheels (left and right).
     */
    private final DifferentialDriveKinematics mDriveKinematics;
    /**
     * Left Motor PID Controller
     */
    private final PIDController mLeftController;
    /**
     * Right Motor PID Controller
     */
    private final PIDController mRightController;
    /**
     * The trajectory to execute. At this time this class reaches out and gets the trajectory. Since
     * the trajectory needs to be restarted every time, this is not final and is reloaded in the init method.
     */
    private Trajectory mTrajectory;
    /**
     * A history of the speeds of the wheels this module has calculated for the trajectory
     */
    private DifferentialDriveWheelSpeeds mPrevSpeeds;
    /**
     * The time, in seconds. This may not reflect realtime.
     */
    private double mPrevTime;

    /**
     * Unique identifier for this object. This should get reinitialized on each initialize call
     *
     */
    private UUID mID;
    private Trajectory.State initialState;

    /**
     * Default constructor. This will instantiate the variables that are not dependent on the init
     * state of autonomous. For those components, those will be initialized in the initialized method
     */
    public BaseAutonController() {
        mFollower = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
        mRightController = new PIDController(2,0, 0);
        mLeftController = new PIDController(2, 0, 0);
        mTimer = new Timer();
        mDriveKinematics = new DifferentialDriveKinematics(Settings.kTrackWidthMeters);
        SmartDashboard.putNumber("trajectory-seconds",-1);
    }

    /**
     * Method that should be called when the autonomous state is initialized. This is so that the overall
     * state can be restarted, should the autonomous be run multiple times. (Something that only really happens
     * at home).
     */
    public void initialize() {
        mID = UUID.randomUUID();
        mTimer.reset();
        mTimer.start();
        mLeftController.reset();
        mRightController.reset();
        mPrevTime = -1;
        mTrajectory = TrajectoryCommandUtils.getJSONTrajectory();
//        Trajectory trajectory = TrajectoryCommandUtils.getJSONTrajectory();
//        Transform2d transform = getRobotPose().minus(trajectory.getInitialPose());
//        mTrajectory = trajectory.transformBy(transform);
        System.out.println(mTrajectory.getInitialPose());
        initialState = mTrajectory.sample(0);
        mPrevSpeeds = new DifferentialDriveWheelSpeeds(0,0);
        mLeftController.reset();
        mRightController.reset();

        SmartDashboard.putNumber("Trajectory Total Time in Seconds", mTrajectory.getTotalTimeSeconds());
    }
    @Override
    protected void updateImpl() {
        execute();
    }
    private static int EXEC_COUNT = 1;
    private static boolean HAS_FINISHED = false;

    public void execute_simple_deadreckon() {
        SmartDashboard.putNumber("trajectory-seconds",mTrajectory.getTotalTimeSeconds());
        double curTime = mTimer.get();
        double dT = curTime - mPrevTime;

        if (mPrevTime < 0) {
            updateDriveTrain(new ImmutablePair<Double,Double>(0d,0d));
            mPrevTime = curTime;
            return;
        }

        double target_speed_meters_per_second = 20;
        if(!isFinished()) {
            DifferentialDriveWheelSpeeds targetWheelSpeeds = new DifferentialDriveWheelSpeeds(target_speed_meters_per_second,target_speed_meters_per_second);
            perform_execute(curTime,dT,calculateActualSpeeds(),targetWheelSpeeds);
        }
    }
    /**
     * Method to perform the actual traversal. This should run a single step. A step will be defined as
     * a sequence of execution between some delta time from the previous execution. This method is expected to be
     * called multiple times until the robot traverses the entire Trajectory or until autonmous runs out of time.
     */
    public void execute() {

        if(false) {
            execute_simple_deadreckon();
            return;
        }


        double curTime = mTimer.get();
        double dT = curTime - mPrevTime;


        if (mPrevTime < 0) {
            updateDriveTrain(new ImmutablePair<Double,Double>(0d,0d));
            mPrevTime = curTime;
            return;
        }

        Pose2d robotPose = getRobotPose();
        Trajectory.State sample = mTrajectory.sample(curTime);

        DifferentialDriveWheelSpeeds targetWheelSpeeds = getTargetWheelSpeeds(sample, robotPose);
        DifferentialDriveWheelSpeeds actualSpeeds = calculateActualSpeeds();
        logDataToSmartDashboard(dt, sample, targetWheelSpeeds, actualSpeeds);

        perform_execute(curTime, dT, actualSpeeds, targetWheelSpeeds);
    }

    private void perform_execute(double curTime, double dT, DifferentialDriveWheelSpeeds actualSpeeds, DifferentialDriveWheelSpeeds targetWheelSpeeds) {
        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
        MutablePair<Double,Double> output = new MutablePair<>();

        double leftSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        double leftFeedforward = calculateFeedsForward(leftSetpoint, mPrevSpeeds.leftMetersPerSecond, dT);
        double rightFeedforward = calculateFeedsForward(rightSetpoint, mPrevSpeeds.rightMetersPerSecond, dT);

        output.left = calculateOutputFromFeedForward(leftFeedforward, mLeftController, actualSpeeds.leftMetersPerSecond, targetWheelSpeeds.leftMetersPerSecond);
        output.right = calculateOutputFromFeedForward(rightFeedforward, mRightController, actualSpeeds.rightMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);
        updateDriveTrain(output);
        mPrevSpeeds = targetWheelSpeeds;
        mPrevTime = curTime;
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

    /**
     * Helper method to get constant speed. This will check to see if we are finished with the
     * trajectory.If it is, it will return 0 speed. Otherwise it will return full speed straight.
     * @return
     */
    private ImmutablePair<Double,Double> getConstantSpeed() {
        double speed = isFinished() ? 0.0 : 7.0;
        return new ImmutablePair<>(speed,speed);
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
    private double calculateOutputFromFeedForward(double pFeedForward, PIDController pPidController, double pActualSpeeds, double pSetPoint) {
        return pFeedForward + pPidController.calculate(pActualSpeeds, pSetPoint);
    }

    /**
     * Method to calcualte the actual speeds
     * @return
     *  Return the {@link DifferentialDriveWheelSpeeds}
     */
    private DifferentialDriveWheelSpeeds calculateActualSpeeds() {
        double actualLeftSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
        double actualRightSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s));
        return new DifferentialDriveWheelSpeeds(actualLeftSpeed, actualRightSpeed);
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
    private DifferentialDriveWheelSpeeds getTargetWheelSpeeds(Trajectory.State trajectorySample, Pose2d pRobotPose) {
        return mDriveKinematics.toWheelSpeeds(
                mFollower.calculate(pRobotPose, trajectorySample));
    }


    /**
     * Get the current robot opose
     * @return
     *  The robot pose represented as {@link Pose2d}
     */
    private Pose2d getRobotPose() {

        double absX = initialState.poseMeters.getX() + db.drivetrain.get(EDriveData.GET_X_OFFSET_METERS);
        double absY = initialState.poseMeters.getY() + db.drivetrain.get(EDriveData.GET_Y_OFFSET_METERS);
        Rotation2d r2d = new Rotation2d(db.drivetrain.get(EDriveData.ACTUAL_HEADING_RADIANS));
        Pose2d robotPose = new Pose2d(new Translation2d(absX,absY),r2d);

        return robotPose;
    }

    /**
     * Method to update the motors
     * @param pOutput
     *  The voltage values to set the motors
     */
    private void updateDriveTrain(Pair<Double,Double> pOutput) {
     //   System.out.println("BaseAutonController: Setting desired voltage= ["+pOutput+"]");
        SmartDashboard.putNumber("DriveTrainVoltage-Left",pOutput.getLeft());
        SmartDashboard.putNumber("DriveTrainVoltage-Right",pOutput.getRight());
        db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, pOutput.getLeft());
        db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, pOutput.getRight());
    }

    /**
     * Method to check if the trajectory is finished
     * @return
     *  true if the trajectory traversal is complete, false otherwise
     */
    public boolean isFinished() {
        return mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds());
    }

    /**
     * Method to stop
     * @param interrupted
     *  If true, this will force stop
     */
    public void end(boolean interrupted) {
        mTimer.stop();
        if (interrupted) {
            db.drivetrain.set(EDriveData.DESIRED_LEFT_VOLTAGE, 0);
            db.drivetrain.set(EDriveData.DESIRED_RIGHT_VOLTAGE, 0);
        }
    }
}