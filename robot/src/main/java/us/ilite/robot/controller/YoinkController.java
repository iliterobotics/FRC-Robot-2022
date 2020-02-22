package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private double mStartTime;

    private boolean isFinished(double pNow, Path pPath) {
        return BobUtils.getIndexForCumulativeTime(pPath, pNow, mStartTime) == pPath.getPath().length - 1;
    }

    public YoinkController(double pNow) {
        mStartTime = pNow;
        setActivePath(new GoToTrenchFromLine());
        System.out.println("AUTONOMOUS ROUTINE HAS BEEN STARTED");
    }

    @Override
    public void updateImpl(double pNow) {
        if (isFinished(pNow, new GoToTrenchFromLine())) {
            setActivePath(new ReturnToShooting());
            mPathFollower.reverse();
            System.out.println("ROBOT IS AT TRENCH");
        }
        if (isFinished(pNow, new ReturnToShooting())) {
            setActivePath(new PickUpBallsFromGenerator());
            System.out.println("ROBOT IS BACK TO THE INITIATION LINE");
        }
        if (isFinished(pNow, new PickUpBallsFromGenerator())) {
            setActivePath(new FinalShots());
            System.out.println("ROBOT IS AT GENERATOR");
        }
        System.out.println("AUTONOMOUS ROUTINE HAS BEEN FINISHED");
        super.updateImpl(pNow);
    }
}
