package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.feet_to_meters;

public class ThreeBallTrajectoryAuton extends BaseAutonController {
    public static Pose2d
            ROBOT_START = new Pose2d(feet_to_meters(25.016), feet_to_meters(5.724), Rotation2d.fromDegrees(270d)),
            SECOND_BALL = new Pose2d(feet_to_meters(25.380), feet_to_meters( 2.733), Rotation2d.fromDegrees(270)),
            SHOT_AGAINST_HUB = new Pose2d(feet_to_meters(25.866), feet_to_meters(10.343), Rotation2d.fromDegrees(-81)),
            THIRD_BALL = new Pose2d(feet_to_meters(16.602), feet_to_meters(6.029), Rotation2d.fromDegrees(-25)),
            INTERMEDIATE_STEP = new Pose2d(feet_to_meters(23.073), feet_to_meters(6.983), Rotation2d.fromDegrees(-60))
    ;

    private final Trajectory
        mFirstLeg = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START,SECOND_BALL),super.mTrajectoryConfig.setReversed(false)),
        mSecondLeg = TrajectoryGenerator.generateTrajectory(List.of(SECOND_BALL,SHOT_AGAINST_HUB),super.mTrajectoryConfig.setReversed(true)),
        mThirdLeg = TrajectoryGenerator.generateTrajectory(List.of(SHOT_AGAINST_HUB,INTERMEDIATE_STEP, THIRD_BALL),super.mTrajectoryConfig.setReversed(false)),
        mFourthLeg = TrajectoryGenerator.generateTrajectory(List.of(THIRD_BALL,INTERMEDIATE_STEP, SHOT_AGAINST_HUB),super.mTrajectoryConfig.setReversed(true))
    ;

    private FollowTrajectory mFollower;
    private Timer mTimer = new Timer();
    private boolean mIsFiring = false;

    // TODO Trim off 1 second to fit the 3 ball within 15 seconds
    private final double mFirstLegTime = 2.0; // 2 Seconds
    private final double mSecondLegTime = 5.0; // 3 Seconds
    private final double mShootTime = 6.5; // 1.5 Seconds
    private final double mThirdLegTime = 10.5; // 4 Seconds
    private final double mFourthLegTime = 14.5; // 4 Seconds
    private final double mSecondShootTime = 16.0; // 1.5 Seconds

    public void initialize() {
        mTimer.reset();
        mTimer.start();

        Robot.FIELD.getObject("Tarmac To Top Ball").setTrajectory(mFirstLeg);
        Robot.FIELD.getObject("Top Ball To Shoot").setTrajectory(mSecondLeg);
        Robot.FIELD.getObject("Tarmac To Third Ball").setTrajectory(mThirdLeg);
        Robot.FIELD.getObject("Third Ball To Shoot").setTrajectory(mFourthLeg);

        mFollower = new FollowTrajectory(mFirstLeg, false);
        mFollower.init(mTimer.get());
    }

    public void updateImpl() {
        double time = mTimer.get();

        if (time < mFirstLegTime) { // Intake Cargo #2
            mFollower.update(time);
            intakeCargo();
        } else if (time == mFirstLegTime) {
            mFollower = new FollowTrajectory(mSecondLeg, false);
            mFollower.init(time);
        } else if (time < mSecondLegTime) { // Drive back to shoot
            mFollower.update(time);
            stageBalls();
            setIntakeArmEnabled(false);
        } else if (time < mShootTime) { // Shoot Cargo #1 and #2
            fireCargo();
        } else if (time == mShootTime) {
            mFollower = new FollowTrajectory(mThirdLeg, false);
            mFollower.init(time);
        } else if (time < mThirdLegTime) { // Intake Cargo #3
            mFollower.update(time);
            intakeCargo();
        } else if (time == mThirdLegTime) {
            mFollower = new FollowTrajectory(mFourthLeg, false);
            mFollower.init(time);
        } else if (time < mFourthLegTime) { // Drive back to shoot
            mFollower.update(time);
            stageBalls();
            setIntakeArmEnabled(false);
        } else if (time < mSecondShootTime) { // Shoot Cargo #3
            fireCargo();
        }
    }

    @Override
    public Pose2d getStartPose() {
        return ROBOT_START;
    }
}
