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
    private PowerCellModule.EIntakeState mIntakeState;
    private PowerCellModule.EArmState mArmState;

    public TestController(PowerCellModule pIntake) {
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
        updateArm();
    }

    private void updateIntake() {
        if (Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.INTAKE)) {
            mIntakeState = PowerCellModule.EIntakeState.INTAKE;
        } else if (Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.REVERSE_INTAKE)) {
            mIntakeState = PowerCellModule.EIntakeState.REVERSE;
        } else {
            mIntakeState = PowerCellModule.EIntakeState.STOP;
        }
        switch (mIntakeState) {
            case INTAKE:
               Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , 1.0);
               Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , 1.0);
               Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , 1.0);
               break;
            case REVERSE:
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , -1.0);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , -1.0);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , -1.0);
                break;
            case STOP:
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , 0.0);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_CONVEYOR_TWO_POWER_PCT , 0.0);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT , 0.0);
                break;
        }
    }
    public void updateArm() {
        if (Robot.DATA.operatorinput.isSet(InputMap.OPERATOR.HIGHER_ARM)) {
            mArmState = PowerCellModule.EArmState.ENGAGED;
        } else {
            mArmState = PowerCellModule.EArmState.DISENGAGED;
        }
        switch (mArmState) {
            case ENGAGED:
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 1.0);
                break;
            case DISENGAGED:
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_ARM_STATE , 0.0);
                break;
        }
        //TODO default state
    }
}


