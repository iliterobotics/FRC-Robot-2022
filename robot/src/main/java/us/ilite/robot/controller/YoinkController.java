package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
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
        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" + dt + "\n" +( BobUtils.getPathTotalTime(pPath) )+ "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n ");
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
        activateSerializer(pNow);
        setIntakeArmEnabled(pNow , true);
    }
//    private void startInhaling(double pNow){
//        if(isFinished(pNow, mYoinkTO)){
//            db.powercell.set(EPowerCellData.ARM_ANGLE_deg , 0);
//            db.powercell.set(EPowerCellData.SET_H_pct , 0.5);
//            db.powercell.set(EPowerCellData.SET_V_pct , 0.5);
//        }
//    }
//    private void startShooting(double pNow){
//        if(isFinished(pNow , mYoinkFrom)) {
//            db.flywheel.set(EShooterSystemData.SET_FEEDER_rpm, 6000);
//            db.flywheel.set(EShooterSystemData.SET_BALL_VELOCITY_ft_s, 6000);
//        }
//    }
}
