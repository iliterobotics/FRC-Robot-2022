package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.robot.auto.paths.*;

@Deprecated
public class ShootIntakeController extends BaseAutonController {
    private double mPathDistance = 0d;
    private SimpleSequence mSimpleSequence = new SimpleSequence();
    private SimplePath simplePath;

    public ShootIntakeController() {
        setActivePath(new SimplePath(), false);
    }
    @Override
    protected void updateImpl() {
        super.updateImpl();
        int pathIndex = BobUtils.getIndexForCumulativeTime(simplePath, clock.time(), mPathStartTime);
        if (pathIndex != -1 ){
            mPathDistance = BobUtils.getPathValueForCumulativeTime(simplePath, clock.time(), mPathStartTime, Path.SegmentValue.CENTER_POSITION);
            SmartDashboard.putNumber("PATH DISTANCE", mPathDistance);
            mSimpleSequence.updateSequence(clock.time(), mPathDistance);
        }
    }

}
