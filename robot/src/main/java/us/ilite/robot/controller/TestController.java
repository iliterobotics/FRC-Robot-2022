package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.modules.PowerCellModule;

public class TestController extends AbstractController {

    private PowerCellModule mIntake;
    private Data mData;
    public void update(double pNow) {
        mIntake.setOutputs(pNow);
        mData.powercell.set(EPowerCellData.CURRENT_POWERCELL_STATE, 1.0);
    }
}
