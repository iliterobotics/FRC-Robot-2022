package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import us.ilite.robot.Enums;
import us.ilite.robot.auto.paths.*;

public class NegativeTrajectoryController extends BaseAutonController {
    private Path mFrom = new StraightBackShoot_From();
    private Path mTo = new StraightBackShoot_To();
    private boolean mHasReversed;

    public NegativeTrajectoryController() {
        super(new YoinkTo(),  false);
        mHasReversed = false;
    }

    private boolean isFinished(double pNow, Path pPath) {
        double dt = pNow - mPathStartTime;
        return BobUtils.getIndexForCumulativeTime(pPath, pNow, mPathStartTime) == -1;
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
//        startInhaling(pNow);
//        startShooting(pNow);
    }
    private void startInhaling(double pNow){
        if(isFinished(pNow, mTo)){
            setIntakeArmEnabled(pNow , true);
            activateSerializer(pNow);
        }
    }
    private void startShooting(double pNow){
        if(isFinished(pNow , mFrom)) {
            setFlywheelClosedLoop(Enums.FlywheelSpeeds.FAR);
            setFeederClosedLoop(Enums.FlywheelSpeeds.FAR);
        }
    }
}
