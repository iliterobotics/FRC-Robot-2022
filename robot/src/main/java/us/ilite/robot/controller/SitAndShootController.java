package us.ilite.robot.controller;

import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveModule;

public class SitAndShootController extends BaseAutonController
{
    public SitAndShootController()
    {
        super();
    }

    @Override
    protected void updateImpl(double pNow)
    {
        shoot();
        if( Robot.DATA.powercell.get( EPowerCellData.EXIT_BEAM ) == 3 )
        {
            Robot.DATA.drivetrain.set(EDriveData.L_DESIRED_POS, 60 / (Math.PI * DriveModule.kWheelDiameterInches) * DriveModule.kGearboxRatio );
            Robot.DATA.drivetrain.set(EDriveData.R_DESIRED_POS, 60 / (Math.PI * DriveModule.kWheelDiameterInches) * DriveModule.kGearboxRatio );
        }
    }

    private void shoot()
    {
        if( Robot.DATA.powercell.isSet( EPowerCellData.ENTRY_BEAM ) ||
                Robot.DATA.powercell.isSet( EPowerCellData.H_BEAM ) ||
                Robot.DATA.powercell.isSet( EPowerCellData.EXIT_BEAM ) )
        {
            startShootingLogic( Enums.FlywheelSpeeds.CLOSE );
        }
    }
}
