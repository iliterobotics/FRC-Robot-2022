package us.ilite.robot.commands;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.Limelight;

@Deprecated
public class LimelightTargetLock extends TargetLock {

    private ILog mLog = Logger.createLog(LimelightTargetLock.class);

    private Limelight mLimelight;

    public LimelightTargetLock() {
        super();
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
    }

}