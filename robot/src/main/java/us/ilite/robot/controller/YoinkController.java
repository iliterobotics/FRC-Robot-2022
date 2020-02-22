package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private double mStartTime;
    private Path mYoinkTo = new YoinkTo();
    private Path mYoinkFrom = new YoinkFrom();

    private boolean isFinished(double pNow, Path pPath) {
        return BobUtils.getIndexForCumulativeTime(pPath, pNow, mStartTime) == pPath.getPath().length - 1;
    }

    public YoinkController(double pNow) {
        mStartTime = pNow;
        setActivePath(mYoinkTo);
    }

    @Override
    public void updateImpl(double pNow) {
        if (isFinished(pNow, mYoinkTo)) {
            setActivePath(mYoinkFrom);
            mPathFollower.reverse();
        }
        super.updateImpl(pNow);
    }
}
