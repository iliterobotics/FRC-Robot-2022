package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private double mStartTime;
    private Path mYoinkTo = new YoinkTo();
    private Path mYoinkFrom = new YoinkFrom();

    public YoinkController() {
        super(new YoinkTo(), false);
    }

    private boolean isFinished(double pNow, Path pPath) {
        return BobUtils.getIndexForCumulativeTime(pPath, pNow, mStartTime) == pPath.getPath().length - 1;
    }

    @Override
    public void updateImpl(double pNow) {
        if (isFinished(pNow, mYoinkTo)) {
            setActivePath(mYoinkFrom, true);
        }
        super.updateImpl(pNow);
    }
}
