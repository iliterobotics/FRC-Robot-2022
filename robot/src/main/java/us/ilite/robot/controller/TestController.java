package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.PowerCellModule;

public class TestController extends AbstractController {
    public void update(double pNow) {
       Robot.DATA.powercell.set(EPowerCellData.DESIRED_INTAKE_POWER_PCT , 1.0);
    }
}
