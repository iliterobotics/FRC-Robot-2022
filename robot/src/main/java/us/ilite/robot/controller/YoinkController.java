package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private boolean mHasReversed;

    public YoinkController() {
        super(new YoinkTo(), false);
        mHasReversed = false;
    }

    @Override
    public void updateImpl(double pNow) {
        SmartDashboard.putNumber("INDEX", BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime));
        super.updateImpl(pNow);
        if (!mHasReversed && BobUtils.isFinished(pNow, mActivePath, mPathStartTime)) {
            setNewActivePath(new YoinkFrom(), true);
            mHasReversed = true;

            // Update again since path has changed, follows process of BaseAutonController
            super.updateImpl(pNow);
        }
    }
}
