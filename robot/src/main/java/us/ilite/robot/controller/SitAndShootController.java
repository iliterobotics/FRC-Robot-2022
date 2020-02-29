package us.ilite.robot.controller;

import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

public class SitAndShootController extends BaseAutonController
{

    public SitAndShootController()
    {
        super();
    }

    @Override
    public void updateImpl(double pNow)
    {
        shoot();
    }

    public void shoot()
    {
        if( Robot.DATA.powercell.isSet( EPowerCellData.ENTRY_BEAM ) ||
                Robot.DATA.powercell.isSet( EPowerCellData.H_BEAM ) ||
                Robot.DATA.powercell.isSet( EPowerCellData.EXIT_BEAM ) )
        {
            startShootingLogic( Enums.FlywheelSpeeds.CLOSE );
        }
    }
}
