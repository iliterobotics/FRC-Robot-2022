package us.ilite.robot.controller;

import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.modules.DriveMessage;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.PowerCellModule;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.config.InputMap.DRIVER.*;

public class TestController extends AbstractController {

    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    private PowerCellModule mIntake;
    public TestController( PowerCellModule pIntake) {
        this.mIntake = pIntake;
    }
    public void update(double pNow) {
        updateDrivetrain(pNow);
    }

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        if (throttle == 0.0 && rotate != 0.0) {
            throttle += 0.03;
        }
        var d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
        throttle = d.getThrottle();
        rotate = d.getTurn();
        if (db.driverinput.isSet(SUB_WARP_AXIS) && db.driverinput.get(SUB_WARP_AXIS) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
            throttle *= Settings.Input.kSnailModePercentThrottleReduction;
            rotate *= Settings.Input.kSnailModePercentRotateReduction;
        }
        db.drivetrain.set(THROTTLE, throttle);
        db.drivetrain.set(TURN, rotate);
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

