package us.ilite.robot.controller;

import us.ilite.common.types.EMatchMode;
import us.ilite.robot.modules.FlywheelPrototype;
import us.ilite.robot.modules.Intake;

public class TestController extends AbstractController {

    private Intake mIntake;

    public void update(double pNow) {
        mIntake.setOutputs(pNow);
    }
}
