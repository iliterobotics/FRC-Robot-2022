package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveModule;

public class SitAndShootController extends BaseAutonController
{
    private int mExitBeamBrokenCount;
    private boolean mPreviouslyBroken;
    private boolean mIsBroken;
    public SitAndShootController()
    {
        super();
        mExitBeamBrokenCount = 0;
        mPreviouslyBroken = false;
        mIsBroken = false;
    }

    @Override
    protected void updateImpl(double pNow)
    {
        firingSequence( Enums.FlywheelSpeeds.CLOSE );
        SmartDashboard.putBoolean("EXIT BEAM IS BROKEN", db.powercell.get(EPowerCellData.EXIT_BEAM) == 1.0);
        mIsBroken = db.powercell.get(EPowerCellData.EXIT_BEAM) == 1.0;

        if (mPreviouslyBroken && !mIsBroken) {
            mExitBeamBrokenCount++;
        }

        if(mExitBeamBrokenCount == 3) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.SMART_MOTION);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_POS, 5);
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_POS, 5);
        }
        SmartDashboard.putNumber("EXIT BEAM BROKEN N TIMES", mExitBeamBrokenCount);
        mPreviouslyBroken = mIsBroken;
    }
}
