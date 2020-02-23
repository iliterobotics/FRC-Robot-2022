package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.auto.paths.*;

public class YoinkController extends BaseAutonController {
    private YoinkTo mYoinkTo = new YoinkTo();
    private YoinkFrom mYoinkFrom = new YoinkFrom();
    private boolean mHasReversed;
    private double mIntakingCount = 2 / .02;
    private double mYoinkFromStartTime;

    public YoinkController() {
        super(new YoinkTo(), false);
        mHasReversed = false;
    }

    @Override
    public void updateImpl(double pNow) {
        SmartDashboard.putNumber("INDEX", BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime));
        super.updateImpl(pNow);
        setIntakeArmEnabled(pNow, true);
        activateSerializer(pNow);

        if (BobUtils.isFinished(pNow, mYoinkTo, mPathStartTime)) {
            if (mIntakingCount <= 0 && !mHasReversed) {
                setNewActivePath(mYoinkFrom, true);
                mYoinkFromStartTime = pNow;
                mHasReversed = true;

                // Update again since path has changed, follows process of BaseAutonController
                super.updateImpl(pNow);
            } else {
                mIntakingCount--;
            }
            if (BobUtils.isFinished(pNow, mYoinkFrom, mYoinkFromStartTime)) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TARGET_ANGLE_LOCK);
            }
        }
//        if (!mHasReversed && BobUtils.isFinished(pNow, mYoinkTo, mPathStartTime)) {
//            setNewActivePath(mYoinkFrom, true);
//            mHasReversed = true;
//
//            super.updateImpl(pNow);
//        }
    }
}
