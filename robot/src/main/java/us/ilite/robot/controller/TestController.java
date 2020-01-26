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
        updateIntake();
    }
    private void updateIntake() {
        if(Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.INTAKE)) {
            mIntake.setDesiredIntakeState(PowerCellModule.EIntakeState.INTAKE);
        } else if (Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
            mIntake.setDesiredIntakeState(PowerCellModule.EIntakeState.REVERSE);
        } else {
            mIntake.setDesiredIntakeState(PowerCellModule.EIntakeState.STOP);
        }
    }
}

