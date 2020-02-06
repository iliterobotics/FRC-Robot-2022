package us.ilite.robot.controller;

import org.junit.Assert;
import org.junit.Test;
import static us.ilite.common.config.InputMap.*;

import static java.lang.Math.*;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.config.InputMap;
import us.ilite.robot.BaseTest;
import us.ilite.robot.modules.PowerCellModule;

public class DriveTrainUnitTest extends BaseTest {
    private PowerCellModule mIntake;

    @Test
    public void testDrivetrainControl() {
        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 0.0);
        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
        TestController t = TestController.getInstance();
        t.update(0.0);
        assertNormalizedInputs("0% throttle 100% turn");


        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 1.0);
        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
        t.update(0.0);
        assertNormalizedInputs("100% throttle 100% turn");

        for(int i = 0; i < 100; i++) {
            randomizeAllInputs();
            t.update(0.0);
            assertNormalizedInputs(
                    "THROTTLE-" + nf.format(db.driverinput.get(DRIVER.THROTTLE_AXIS)) +
                    "\tTURN-" + nf.format(db.driverinput.get(DRIVER.TURN_AXIS))
            );
        }
    }

    private void assertNormalizedInputs(String pMessage) {
        Assert.assertTrue(
        "Normalized Driver Inputs not <= 1.0" + pMessage,
                abs(db.drivetrain.get(DESIRED_THROTTLE)) + abs(db.drivetrain.get(DESIRED_TURN)) <= 1.0
        );
    }

}
