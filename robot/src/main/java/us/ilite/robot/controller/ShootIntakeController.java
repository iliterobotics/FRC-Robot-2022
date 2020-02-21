package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.Robot;
import us.ilite.robot.auto.paths.BobUtils;
import us.ilite.robot.auto.paths.SimpleSequence;
import us.ilite.robot.auto.paths.T_LINE_10_FT;

public class ShootIntakeController extends BaseAutonController {
    private double mPathDistance = 0d;
    private SimpleSequence mSimpleSequence = new SimpleSequence();

    public ShootIntakeController() {
        super(new T_LINE_10_FT());
    }
    @Override
    protected void updateImpl(double pNow) {
        super.updateImpl(pNow);
        int pathIndex = BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime);
        if (pathIndex != -1 ){
            mPathDistance = mActivePath.getPath()[pathIndex][7];
            mSimpleSequence.updateSequence(pNow, mPathDistance);
        }
    }

}
