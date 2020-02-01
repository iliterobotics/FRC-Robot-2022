package us.ilite.robot.controller;

import junit.framework.ComparisonFailure;
import org.junit.Assert;
import org.junit.Test;
import us.ilite.common.Data;
import static us.ilite.common.config.InputMap.*;

import static java.lang.Math.*;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.config.InputMap;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class DriveTrainUnitTest {
    private final Data db = Robot.DATA;


    @Test
    public void testDrivetrainControl() {
        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 0.0);
        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
        TestController t = new TestController();
        t.updateDrivetrain(0.0);
        assertNormalizedInputs("0% throttle 100% turn");


        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 1.0);
        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
        t.updateDrivetrain(0.0);
        assertNormalizedInputs("100% throttle 100% turn");

        for(int i = 0; i < 100; i++) {
            double turn = random() * (random() > 0.5 ? 1.0 : -1.0);
            double throttle = random() * (random() > 0.5 ? 1.0 : -1.0);
            db.driverinput.set(DRIVER.THROTTLE_AXIS, throttle);
            db.driverinput.set(DRIVER.TURN_AXIS, turn);
            t.updateDrivetrain(0.0);
            assertNormalizedInputs(
                    "THROTTLE-" + nf.format(throttle) +
                    "\tTURN-" + nf.format(turn)
            );
        }
    }

    private void assertNormalizedInputs(String pMessage) {
//        try {
            Assert.assertTrue(
                    "Normalized Driver Inputs not <= 1.0" + pMessage,
                    abs(db.drivetrain.get(DESIRED_THROTTLE)) + abs(db.drivetrain.get(DESIRED_TURN)) <= 1.0
            );
//        } catch(ComparisonFailure e) {
//            System.out.println(e.getMessage());
//        }
    }

    private static final NumberFormat nf = new DecimalFormat("0.00");
}
