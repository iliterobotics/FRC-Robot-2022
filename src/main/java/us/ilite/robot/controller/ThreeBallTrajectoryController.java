package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.FollowTrajectory;

import java.util.List;

import static us.ilite.common.lib.util.Units.feet_to_meters;

public class ThreeBallTrajectoryController extends BaseAutonController {
    //ROBOT POSES
    public static final Pose2d ROBOT_START = new Pose2d(new Translation2d(Units.feet_to_meters(28.415), Units.feet_to_meters(17.214)), Rotation2d.fromDegrees(67.641));
    public static final Pose2d FIRST_BALL = new Pose2d(new Translation2d(Units.feet_to_meters(34.858), Units.feet_to_meters(20.018)), Rotation2d.fromDegrees(0d));

    //ROBOT TRAJECTORIES
    private Trajectory mFirstBallPath = TrajectoryGenerator.generateTrajectory(List.of(ROBOT_START, FIRST_BALL), super.mTrajectoryConfig.setReversed(false));
    private Trajectory mFirstReturnPath = TrajectoryGenerator.generateTrajectory(List.of(FIRST_BALL, ROBOT_START), super.mTrajectoryConfig.setReversed(true));

    //command that will actually drive the robot
    private FollowTrajectory mFollower;

    //ROBOT PATH TIME ENDS
    private double mShotTime = 0.5;
    private double m2ndBallLegTime = 0;
    private double mReturnShootLeg = 0;

    private Timer mTimer;

    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();;
        m2ndBallLegTime = mShotTime + mFirstBallPath.getTotalTimeSeconds();
        mReturnShootLeg = m2ndBallLegTime + mFirstReturnPath.getTotalTimeSeconds();
        mFollower = new FollowTrajectory(mFirstBallPath, false);
        mFollower.init(mTimer.get());
    }
    public void updateImpl() {
        double time = mTimer.get();
        if (time < mShotTime) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET_ODOMETRY);
            db.drivetrain.set(EDriveData.X_DESIRED_ODOMETRY_METERS, mFirstBallPath.getInitialPose().getX());
            db.drivetrain.set(EDriveData.Y_DESIRED_ODOMETRY_METERS, mFirstBallPath.getInitialPose().getY());
            fireCargo();
        }
        else if (time < m2ndBallLegTime) { // Move to Ball #1 (in the direction of human player station)
            intakeCargo();
            mFollower.update(time);
        } else if (time < m2ndBallLegTime + 0.1) {
            intakeCargo();
            mFollower = new FollowTrajectory(mFirstBallPath, false);
            mFollower.init(time);
        } else if (time < mReturnShootLeg) {
            intakeCargo();
            mFollower.update(time);
        } else {
            fireCargo();
        }
    }

    public Pose2d getStartPose() {
        return ROBOT_START;
    }
}
