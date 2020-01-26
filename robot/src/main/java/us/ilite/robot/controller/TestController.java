package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.PowerCellModule;

public class TestController extends AbstractController {
    private PowerCellModule mIntake;
    public TestController( PowerCellModule pIntake) {
        this.mIntake = pIntake;
    }
    public void update(double pNow) {
        updatePowerCellModule();
    }
    public void updatePowerCellModule(){
        if ( Robot.DATA.operatorinput.isSet(ELogitech310.X_BTN) ) {
                mIntake.startIntaking();
        }
    }
}
