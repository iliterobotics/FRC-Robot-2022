package us.ilite.robot.controller;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;

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
     * Flag indiciating if this run should use PID in it's calculation
     */
    private final boolean mUsePid;
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
     * Default constructor. This will instantiate the variables that are not dependent on the init
     * state of autonomous. For those components, those will be initialized in the initialized method
     */
    public BaseAutonController() {
        mUsePid = true;
        mFollower = new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta);
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kP, Settings.kA);
        mRightController = new PIDController(Settings.kP, 0, 0);
        mLeftController = new PIDController(Settings.kP, 0, 0);

        mTimer = new Timer();
        mDriveKinematics = new DifferentialDriveKinematics(Settings.kTrackwidthMeters);


    }

    /**
     * Method that should be called when the autonmous state is initialized. This is so that the overall
     * state can be restarted, should the autonomous be run multiple times. (Something that only really happens
     * at home).
     */
    public void initialize() {
        mTimer.reset();
        mTimer.start();
        mLeftController.reset();
        mRightController.reset();
        mPrevTime = -1;
        mTrajectory = TrajectoryCommandUtils.getTrajectory();
        var initialState = mTrajectory.sample(0);
        mPrevSpeeds =
                mDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        mTimer.reset();
        mTimer.start();
        if (mUsePid) {
            mLeftController.reset();
            mRightController.reset();
        }

    }
    @Override
    protected void updateImpl() {
        execute();
    }

    /**
     * Method to perform the actual traversal. This should run a single step. A step will be defined as
     * a sequence of execution between some delta time from the previous execution. This method is expected to be
     * called multiple times until the robot traverses the entire Trajectory or until autonmous runs out of time.
     */
    private void execute() {
        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PATH_FOLLOWING_RAMSETE);
        double curTime = mTimer.get();
        double dt = curTime - mPrevTime;

        if (mPrevTime < 0) {
            updateDriveTrain(new ImmutablePair<Double,Double>(0d,0d));
            mPrevTime = curTime;
            return;
        }
        Pose2d robotPose = getRobotPose();
        var targetWheelSpeeds = getTargetWheelSpeeds(curTime, robotPose);

        MutablePair<Double,Double> output = new MutablePair<>();

        if (mUsePid) {
            DifferentialDriveWheelSpeeds actualSpeeds = calculateActualSpeeds(targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);

            double leftFeedforward = calculateFeedsForward(targetWheelSpeeds.leftMetersPerSecond, mPrevSpeeds.leftMetersPerSecond, dt);
            double rightFeedforward = calculateFeedsForward(targetWheelSpeeds.rightMetersPerSecond, mPrevSpeeds.rightMetersPerSecond, dt);

            output.left = calculateOutputFromFeedForward(leftFeedforward, mLeftController,actualSpeeds.leftMetersPerSecond,leftSpeedSetpoint);
            output.right = calculateOutputFromFeedForward(rightFeedforward, mRightController,actualSpeeds.rightMetersPerSecond,targetWheelSpeeds.rightMetersPerSecond);

        } else {
            output.left = targetWheelSpeeds.leftMetersPerSecond;
            output.right = targetWheelSpeeds.rightMetersPerSecond;
        }

        updateDriveTrain(output);
        mPrevSpeeds = targetWheelSpeeds;
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
    private double calculateOutputFromFeedForward(double pFeedForward, PIDController pPidController, double pActualSpeeds, double pSetPoint) {
        return pFeedForward + pPidController.calculate(pActualSpeeds, pSetPoint);
    }

    /**
     * Method to calcualte the actual speeds from the set points
     * @param pLeftSpeedSetpoint
     *  The left setpoint
     * @param pRightSpeedSetpoint
     *  The right setpoint
     * @return
     *  Return the {@link DifferentialDriveWheelSpeeds}
     */
    private DifferentialDriveWheelSpeeds calculateActualSpeeds(double pLeftSpeedSetpoint, double pRightSpeedSetpoint) {

        double actualLeftSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
        double actualRightSpeed = Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s));
        return new DifferentialDriveWheelSpeeds(actualLeftSpeed, actualRightSpeed);
    }

    /**
     * Method to get the target wheel speeds
     * @param pCurTime
     *  The current time in seconds
     * @param pRobotPose
     *  The current robot pose
     * @return
     *  The robot wheel speeds
     */
    private DifferentialDriveWheelSpeeds getTargetWheelSpeeds(double pCurTime, Pose2d pRobotPose) {
        return mDriveKinematics.toWheelSpeeds(
                mFollower.calculate(pRobotPose, mTrajectory.sample(pCurTime)));
    }


    /**
     * Get the current robot opose
     * @return
     *  The robot pose represented as {@link Pose2d}
     */
    private Pose2d getRobotPose() {
        Rotation2d r2d = new Rotation2d(Units.degrees_to_radians(db.drivetrain.get(EDriveData.DELTA_HEADING)));
        Pose2d robotPose = new Pose2d(db.drivetrain.get(EDriveData.GET_X_OFFSET), db.drivetrain.get(EDriveData.GET_Y_OFFSET), r2d);
        return robotPose;
    }

    /**
     * Method to update the motors
     * @param pOutput
     *  The voltage values to set the motors
     */
    private void updateDriveTrain(Pair<Double,Double> pOutput) {
        System.out.println("BaseAutonController: Setting desired voltage= ["+pOutput+"]");
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