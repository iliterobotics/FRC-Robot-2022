package us.ilite.robot.commands;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

import static us.ilite.common.types.ELimelightData.*;

@Deprecated
public class TargetLock implements ICommand {

    private static final double kTURN_POWER = 0.2;
    private static final int kAlignCount = 10;
    private static final double kTargetAreaScalar = 1.0;
    private static final double kAllowableTargetLockError = 0.1;

    private boolean mEndOnAlignment = true;
    private int mAlignedCount;
    private boolean mHasAcquiredTarget;

    public TargetLock() {
    }

    @Override
    public void init(double pNow) {
        mHasAcquiredTarget = false;
        mAlignedCount = 0;

        Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TARGET_ANGLE_LOCK);
        Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.0);
    }

    @Override
    public boolean update(double pNow) {
        RobotCodex<ELimelightData> currentData = Robot.DATA.limelight;

        Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, Robot.DATA.operatorinput.get(InputMap.DRIVER.THROTTLE_AXIS) * Settings.Input.kSnailModePercentThrottleReduction);

        if (currentData.isSet(TV) && currentData.isSet(TX)) {
            mHasAcquiredTarget = true;
            mAlignedCount++;
            if (mEndOnAlignment && Math.abs(currentData.get(TX)) < kAllowableTargetLockError && mAlignedCount > kAlignCount) {
                return true;
            }

            // If we've already seen the target and lose tracking, exit.
        } else if (mHasAcquiredTarget && !currentData.isSet(TV)) {
            return true;
        }

        //command has not completed
        return false;
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.NORMAL);
    }

    public void setEndOnAlignment(boolean bool) {
        mEndOnAlignment = bool;
    }
}