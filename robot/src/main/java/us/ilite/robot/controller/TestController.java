package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.types.EIntake;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.modules.FlywheelPrototype;
import us.ilite.robot.modules.Intake;

public class TestController extends AbstractController {

    private Intake mIntake;
    private Data mData;
    public void update(double pNow) {
        mIntake.setOutputs(pNow);
        mData.intake.set(EIntake.TARGET_INTAKE_STATE, (double) EIntake.TARGET_INTAKE_STATE.ordinal());
    }
}
