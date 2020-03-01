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
    public SitAndShootController()
    {
        super();
        mExitBeamBrokenCount = 0;
        mPreviouslyBroken = false;
        mIsBroken = false;
    }

    @Override
    protected void updateImpl(double pNow) {
        firingSequence(Enums.FlywheelSpeeds.INITIATION_LINE, Field2020.FieldElement.OUTER_GOAL);
        mIsBroken = db.powercell.get(EPowerCellData.EXIT_BEAM) == 1.0;

        if (mPreviouslyBroken && !mIsBroken) {
            mExitBeamBrokenCount++;
        }

        if(mExitBeamBrokenCount == 3) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.SMART_MOTION);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_POS, 10);
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_POS, 10);
        }
        SmartDashboard.putNumber("EXIT BEAM BROKEN N TIMES", mExitBeamBrokenCount);
        mPreviouslyBroken = mIsBroken;
    }
}
