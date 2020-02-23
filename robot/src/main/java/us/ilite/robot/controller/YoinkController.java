package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private Path mYoinkTo = new YoinkTo();
    private boolean mHasReversed;

    public YoinkController() {
        super(new YoinkTo(), false);
        mHasReversed = false;
    }

    @Override
    public void updateImpl(double pNow) {
        super.updateImpl(pNow);
        if (!mHasReversed && BobUtils.isFinished(pNow, mActivePath, mPathStartTime)) {
            setNewActivePath(new YoinkFrom(), true);
            mHasReversed = true;

            // Update again since path has changed, follows process of BaseAutonController
            super.updateImpl(pNow);
        }
    }
}
