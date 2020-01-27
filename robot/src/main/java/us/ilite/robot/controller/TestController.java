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
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT, (double) PowerCellModule.EIntakeState.INTAKE.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT, (double) PowerCellModule.EIntakeState.INTAKE.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT, (double) PowerCellModule.EIntakeState.INTAKE.ordinal());
        } else if (Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
//            mIntake.setDesiredIntakeState(PowerCellModule.EIntakeState.REVERSE);
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT, (double) PowerCellModule.EIntakeState.REVERSE.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT, (double) PowerCellModule.EIntakeState.REVERSE.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT, (double) PowerCellModule.EIntakeState.REVERSE.ordinal());
        } else {
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT, (double) PowerCellModule.EIntakeState.STOP.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT, (double) PowerCellModule.EIntakeState.STOP.ordinal());
            Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT, (double) PowerCellModule.EIntakeState.STOP.ordinal());
        }
    }
}

