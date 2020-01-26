package us.ilite.robot.commands;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ETrackingType;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveModule;
import us.ilite.robot.modules.IThrottleProvider;
import us.ilite.robot.modules.Limelight;

public class LimelightTargetLock extends TargetLock {

    private ILog mLog = Logger.createLog(LimelightTargetLock.class);

    private Limelight mLimelight;


//    public LimelightTargetLock(Drive pDrive, Limelight pLimelight, double pAllowableError, ETrackingType pTrackingType, IThrottleProvider pThrottleProvider) {
//        super(pDrive, pAllowableError, pTrackingType, pLimelight, pThrottleProvider);
//        this.mLimelight = pLimelight;
//        mLimelight.setTracking(pTrackingType);
//        mLog.error("STARTED LIMELIGHT TARGET LOCK");
//    }

    public LimelightTargetLock(DriveModule pDrive, Limelight pLimelight, double pAllowableError, IThrottleProvider pThrottleProvider, boolean pEndOnAlignment) {
        super(pDrive, pAllowableError, pLimelight, pThrottleProvider, pEndOnAlignment);

        this.mLimelight = pLimelight;
        mLog.error("STARTED LIMELIGHT TARGET LOCK");
    }

    @Override
    public void init(double pNow) {
        super.init(pNow);
        mLog.error("STARTED LIMELIGHT TARGET LOCK");
    }

    public void shutdown(double pNow) {
        super.shutdown(pNow);
        mLog.warn("SHUT DOWN LIMELIGHT TARGET LOCK");
        Robot.DATA.limelight.set(ELimelightData.TRACKING_TYPE , (double) ETrackingType.NONE.ordinal());
    }

}
