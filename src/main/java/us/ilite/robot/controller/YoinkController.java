package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private YoinkTo mYoinkTo = new YoinkTo();
    private YoinkFrom mYoinkFrom = new YoinkFrom();
    private boolean mHasReversed;
    private double mIntakingCount = 1 / .02;
    private double mYoinkFromStartTime;

    public YoinkController() {
        super(new YoinkTo(), false);
        mHasReversed = false;
    }

    @Override
    public void updateImpl() {
        SmartDashboard.putNumber("INDEX", BobUtils.getIndexForCumulativeTime(mActivePath, clock.now(), mPathStartTime));
        super.updateImpl();
//        setIntakeArmEnabled(pNow, true);
//        activateSerializer(pNow);

        if (BobUtils.isFinished(clock.now(), mYoinkTo, mPathStartTime)) {
            if (mIntakingCount <= 0 && !mHasReversed) {
                setNewActivePath(mYoinkFrom, true);
                mYoinkFromStartTime = clock.now();
                mHasReversed = true;

                // Update again since path has changed, follows process of BaseAutonController
                super.updateImpl();
            } else {
                mIntakingCount--;
            }
            if (BobUtils.isFinished(clock.now(), mYoinkFrom, mYoinkFromStartTime)) {
                setTargetTracking(true);
            }
        }
    }
}
