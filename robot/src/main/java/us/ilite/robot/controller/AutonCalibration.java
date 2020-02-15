package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.auto.paths.BobUtils;
import us.ilite.robot.auto.paths.T_LINE_10_FT;
import us.ilite.robot.auto.paths.T_LINE_F10FT_L90DEG_F5FT_R5FT;
import us.ilite.robot.modules.EDriveState;

public class AutonCalibration extends BaseAutonController {

    private Path mPath = new T_LINE_F10FT_L90DEG_F5FT_R5FT();
    private final Distance mPathTotalDistance;
    private final double mMaxAllowedPathTime;

    public AutonCalibration() {
        mPathTotalDistance = BobUtils.getPathTotalDistance(mPath);
        mMaxAllowedPathTime = BobUtils.getPathTotalTime(mPath) + 0.1;
        mActivePath = mPath;
        e();
        System.out.println("==== RUNNING AUTONOMOUS PATH ====");
        System.out.println("Path: " + mActivePath.getClass().getSimpleName());
        System.out.println("Time (s): " + mMaxAllowedPathTime);
        System.out.println("Dist (ft): " + mPathTotalDistance);
        e();
    }

    @Override
    protected void updateImpl(double pNow) {
        if(mPathStartTime == 0) {
            mPathStartTime = pNow;
        }

        // Add a time check to prevent errors when things go wrong
        if(mActivePath != null && pNow - mPathStartTime <= mMaxAllowedPathTime) {
            int index = BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime);
            if(index >= 0) {
                db.drivetrain.set(EDriveData.STATE, EDriveState.PATH_FOLLOWING_BASIC);
                db.drivetrain.set(EDriveData.L_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.LEFT_VELOCITY));
                db.drivetrain.set(EDriveData.R_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.RIGHT_VELOCITY));
            } else {
                e();
                System.out.println("==== SUCCESSFULLY END AUTONOMOUS PATH ====");
                e();
                mActivePath = null;
            }
        } else if(mActivePath != null && pNow - mPathStartTime > mMaxAllowedPathTime) {
            e();
            System.out.println("==== END AUTONOMOUS PATH DUE TO TIME OVERRUN ====");
            e();
        }
    }

    private static final void e() {
        System.out.println("================================================");
    }
}
