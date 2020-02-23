package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Enums;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private Path mYoinkFrom = new YoinkFrom();
    private Path mYoinkTO = new YoinkTo();
    private boolean mHasReversed;

    public YoinkController() {
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
        startInhaling(pNow);
        startShooting(pNow);
    }
    private void startInhaling(double pNow){
        if(isFinished(pNow, mYoinkTO)){
            setIntakeArmEnabled(pNow , true);
            activateSerializer(pNow);
        }
    }
    private void startShooting(double pNow){
        if(isFinished(pNow , mYoinkFrom)) {
           setFlywheelClosedLoop(Enums.FlywheelSpeeds.FAR);
           setFeederClosedLoop(Enums.FlywheelSpeeds.FAR);
        }
    }
}
