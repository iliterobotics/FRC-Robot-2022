package us.ilite.robot.controller;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class BaseAutonController extends AbstractController {

    private Timer mTimer;
    private boolean mUsePid;
    private Trajectory mTrajectory;
    private Supplier<Pose2d> mPoses;
    private RamseteController mFollower;
    private SimpleMotorFeedforward mFeedforward;
    private DifferentialDriveKinematics mDriveKinematics;
    private DifferentialDriveWheelSpeeds m_speeds;
    private PIDController mLeftController;
    private PIDController mRightController;
    private BiConsumer<Double, Double> mOutput;
    private DifferentialDriveWheelSpeeds mPrevSpeeds;
    private double mPrevTime;

    public BaseAutonController() {
        mTimer = new Timer();
        mPrevTime = -1;
        var initialState = mTrajectory.sample(0);
        mPrevSpeeds =
                mDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_speeds = new DifferentialDriveWheelSpeeds(Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)
        ), Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s)));
        mTimer.reset();
        mTimer.start();
        mUsePid = true;
        mFollower = new RamseteController();
        mFeedforward = new SimpleMotorFeedforward(Settings.kS, Settings.kP, Settings.kA);
        mRightController = new PIDController(Settings.kP, 0, 0);
        mLeftController = new PIDController(Settings.kP, 0, 0);
        if (mUsePid) {
            mLeftController.reset();
            mRightController.reset();
        }
        mDriveKinematics = new DifferentialDriveKinematics(Settings.kTrackwidthMeters);
    }
    @Override
    protected void updateImpl() {

    }
    public void execute() {
        double curTime = mTimer.get();
        double dt = curTime - mPrevTime;

        if (mPrevTime < 0) {
            mOutput.accept(0.0, 0.0);
            mPrevTime = curTime;
            return;
        }

        //Setup the differential drive wheel speeds class
        double leftSpeeds = Units.feet_to_meters(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s));
        double rightSpeeds = Units.feet_to_meters(db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s));
        DifferentialDriveWheelSpeeds driveSpeeds = new DifferentialDriveWheelSpeeds(leftSpeeds, rightSpeeds);
        double leftSpeedSetpoint = driveSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = driveSpeeds.rightMetersPerSecond;

        double leftOutput = 0;
        double rightOutput = 0;

        if (mUsePid) {
            double leftFeedforward =
                    mFeedforward.calculate(
                            leftSpeedSetpoint, (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    mFeedforward.calculate(
                            rightSpeedSetpoint, (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt);

            leftOutput =
                    leftFeedforward
                            + mLeftController.calculate(m_speeds.leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput =
                    rightFeedforward
                            + mRightController.calculate(
                            m_speeds.rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }
        mOutput.accept(leftOutput, rightOutput);
        mPrevSpeeds = driveSpeeds;
        mPrevTime = curTime;
    }
}