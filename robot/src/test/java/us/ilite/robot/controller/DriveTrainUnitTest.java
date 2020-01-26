package us.ilite.robot.controller;

import org.junit.Test;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;

public class DriveTrainUnitTest {


    @Test
    public void testDrivetrainControl() {
        Data db = Robot.DATA;
        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 0.0);
        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
        TestController t = new TestController();
        t.updateDrivetrain(0.0);
        StringBuilder sb = new StringBuilder();
        sb.append("OUTPUT_THROTTLE = ").append(db.drivetrain.get(THROTTLE));
        sb.append("\tOUTPUT_TURN = ").append(db.drivetrain.get(TURN));
        System.out.println(sb);
    }

}
