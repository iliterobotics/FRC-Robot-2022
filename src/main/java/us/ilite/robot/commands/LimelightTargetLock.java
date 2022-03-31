package us.ilite.robot.commands;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import us.ilite.common.Data;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.Limelight;

public class LimelightTargetLock {

    private ILog mLog = Logger.createLog(LimelightTargetLock.class);

    private static Limelight mLimelight;
    private static Rotation2d mTargetLockAngle;
    private static final int kAllowableError = 5;
    private static double mDistanceToTarget = 0;

    public LimelightTargetLock() {

    }

    private static Rotation2d calculateTheta() {
        double tx = Robot.DATA.limelight.get(ELimelightData.TX);
        mDistanceToTarget = Robot.DATA.limelight.get(ELimelightData.TARGET_RANGE_in)/12.0;
        mTargetLockAngle = new Rotation2d(Math.atan(tx/mDistanceToTarget) + Robot.DATA.drivetrain.get(EDriveData.ACTUAL_HEADING_DEGREES));
        return mTargetLockAngle;
    }

    public void init(double pNow) {
        super.init(pNow);
    }

    public boolean update(double pNow) {
        super.update(pNow);
        return false;
    }

    public void shutdown(double pNow) {
        super.shutdown(pNow);
    }

}