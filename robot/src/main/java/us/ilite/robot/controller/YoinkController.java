package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private Path mYoinkTo = new YoinkTo();
    private boolean mHasReversed;

    public YoinkController() {
        super(new YoinkTo(), false);
        mHasReversed = false;
    }

    private boolean isFinished(double pNow, Path pPath) {
        double dt = pNow - mPathStartTime;
        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" + dt + "\n" +( BobUtils.getPathTotalTime(pPath) )+ "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n ");
        return BobUtils.getIndexForCumulativeTime(pPath, pNow, mPathStartTime) == -1;
    }

    @Override
    public void updateImpl(double pNow) {
        if (!mHasReversed && isFinished(pNow, mActivePath)) {
            setNewActivePath(new YoinkFrom(), true);
            mHasReversed = true;
        }
        super.updateImpl(pNow);
    }
}
