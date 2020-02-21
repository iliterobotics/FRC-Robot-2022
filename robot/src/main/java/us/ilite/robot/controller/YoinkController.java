package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import us.ilite.robot.auto.paths.BobUtils;
import us.ilite.robot.auto.paths.YoinkFrom;
import us.ilite.robot.auto.paths.YoinkTo;

public class YoinkController extends BaseAutonController {

    private Path mCurrentPath = new YoinkTo();

    private double mStartTime;

    private boolean isAtTrench(double pNow) {
        return BobUtils.getIndexForCumulativeTime(new YoinkTo(), pNow, mStartTime) == new YoinkTo().getPath().length - 1;
    }

    public YoinkController(double pNow) {
        mStartTime = pNow;
        setActivePath(mCurrentPath);
    }

    @Override
    public void updateImpl(double pNow) {
        if (isAtTrench(pNow)) {
            setActivePath(new YoinkFrom());
            mPathFollower.reverse();
        }
        super.updateImpl(pNow);
    }
}
