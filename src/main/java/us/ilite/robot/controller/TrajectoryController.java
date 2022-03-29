package us.ilite.robot.controller;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.commands.FollowTrajectory;

public class TrajectoryController extends BaseAutonController {
    private Timer mTimer;
    private FollowTrajectory mFirstTrajectory = new FollowTrajectory(TrajectoryCommandUtils.getJSONTrajectory(), false);
    public void initialize(Trajectory pTrajectory) {
        //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mFirstTrajectory.init(mTimer.get());
    }
    public void updateImpl() {
        double time = mTimer.get();
        mFirstTrajectory.update(time);
    }
}
