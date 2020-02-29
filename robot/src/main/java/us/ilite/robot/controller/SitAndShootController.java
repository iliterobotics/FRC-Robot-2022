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
    public SitAndShootController()
    {
        super();
        mExitBeamBrokenCount = 0;
    }

    @Override
    protected void updateImpl(double pNow)
    {
//        shoot();
        firingSequence( Enums.FlywheelSpeeds.CLOSE );
        SmartDashboard.putBoolean("EXIT BEAM IS BROKEN", db.powercell.get(EPowerCellData.EXIT_BEAM) == 1.0);
        if (db.powercell.get(EPowerCellData.EXIT_BEAM) == 1.0) {
            mExitBeamBrokenCount++;
        }
        if(mExitBeamBrokenCount >= 3) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.SMART_MOTION);
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_POS, 1);//12 / (Math.PI * DriveModule.kWheelDiameterInches) * DriveModule.kGearboxRatio );
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_POS, 1);//12 / (Math.PI * DriveModule.kWheelDiameterInches) * DriveModule.kGearboxRatio );
        }
        SmartDashboard.putNumber("EXIT BEAM BROKEN N TIMES", mExitBeamBrokenCount);
    }
}
