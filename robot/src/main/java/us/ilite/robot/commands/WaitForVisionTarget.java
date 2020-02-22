package us.ilite.robot.commands;

import us.ilite.common.Data;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.modules.Limelight;

public class WaitForVisionTarget implements ICommand {

    private Data mData;
    private Limelight mLimelight;
    private double mTargetAreaThreshold;

    public WaitForVisionTarget(Data pData, Limelight pLimelight, double pTargetAreaThreshold) {
        mData = pData;
        mLimelight = pLimelight;
        mTargetAreaThreshold = pTargetAreaThreshold;
    }

    @Override
    public void init(double pNow) {
//        mLimelight.setTracking(null);
    }

    @Override
    public boolean update(double pNow) {

        // If target is valid
        if(mData.goaltracking.isSet(ELimelightData.TV)) {
            // If area above threshold exit command
            if(mData.goaltracking.get(ELimelightData.TA) > mTargetAreaThreshold) {
                return true;
            }
        }

        return false;
    }

    @Override
    public void shutdown(double pNow) {
//        mLimelight.setTracking(null);
    }
}
