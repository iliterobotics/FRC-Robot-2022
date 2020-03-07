package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Field2020;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveModule;

public class SitAndShootController extends BaseAutonController
{
    private int mExitBeamBrokenCount;
    private boolean mPreviouslyBroken;
    private boolean mIsBroken;
    private int mCyclesNotShooting = 0;
    private boolean mFinished = false;
    public SitAndShootController() {
        super();
        mExitBeamBrokenCount = 0;
        mPreviouslyBroken = false;
        mIsBroken = false;
    }

    @Override
    protected void updateImpl(double pNow) {
        firingSequence(Enums.FlywheelSpeeds.INITIATION_LINE, Field2020.FieldElement.OUTER_GOAL);
        mIsBroken = db.powercell.isSet(EPowerCellData.EXIT_BEAM);

        if (mPreviouslyBroken && !mIsBroken) {
            mExitBeamBrokenCount++;
        }

        if(mExitBeamBrokenCount == 3 || mCyclesNotShooting >= 10 / .02) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.SMART_MOTION);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_POS, 10);
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_POS, 10);
            setHood(Enums.FlywheelSpeeds.HOME);
        } else {
            mCyclesNotShooting++;
        }
        mPreviouslyBroken = mIsBroken;
        mFinished = getDriveDistance() >= 9;
    }

    private double getDriveDistance() {
        return (db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT) + db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT)) / 2.0;
    }

    public boolean isFinished() {
        return mFinished;
    }
}
