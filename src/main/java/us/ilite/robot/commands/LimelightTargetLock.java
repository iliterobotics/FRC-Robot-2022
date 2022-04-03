package us.ilite.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Field2022;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

import static us.ilite.common.types.ELimelightData.*;
import static us.ilite.common.types.drive.EDriveData.*;

public class LimelightTargetLock implements ICommand {

    private static final int kRPMTolerance = 5;
    private final double mTimeAllotted;

    private Timer mTimer;
    private Enums.EDriveState mPrevDriveState;
    private boolean mShootWhileMove;

    public LimelightTargetLock(double pTimeAllotted, boolean pShootWhileMove) {
        mTimer = new Timer();
        mTimeAllotted = pTimeAllotted;
        mShootWhileMove = pShootWhileMove;
    }

    public void init(double pNow) {
        mTimer.reset();
        mTimer.start();
        mPrevDriveState = Robot.DATA.drivetrain.get(STATE, Enums.EDriveState.class);
    }

    // Calculated based on a quadratic curve with certain desired plot points
    private double distanceBasedThrottle() {
        double distance = Robot.DATA.limelight.get(DISTANCE_TO_TARGET_in);
        return (2.3E-6 * Math.pow(distance + 221.73,2)) - 0.11;
    }

    public boolean update(double pNow) {
        Robot.DATA.limelight.set(TARGET_ID, Field2022.FieldElement.HUB_UPPER.id());
        Robot.DATA.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);

        if (mShootWhileMove) {
            Robot.DATA.drivetrain.set(DESIRED_THROTTLE_PCT, distanceBasedThrottle());
        }

        if (mTimer.get() >= mTimeAllotted) {
            shutdown(pNow);
            return true;
        } else {
            return Robot.DATA.limelight.isSet(TX) && Robot.DATA.drivetrain.get(L_ACTUAL_VEL_RPM) <= kRPMTolerance;
        }
    }

    public void shutdown(double pNow) {
        Robot.DATA.drivetrain.set(STATE, mPrevDriveState);
        Robot.DATA.limelight.set(TARGET_ID, Field2022.FieldElement.CAMERA.id());
    }

}