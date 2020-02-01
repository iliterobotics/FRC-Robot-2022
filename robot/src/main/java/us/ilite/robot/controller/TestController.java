package us.ilite.robot.controller;

import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveMessage;
import us.ilite.robot.modules.EDriveState;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.config.InputMap.DRIVER.*;

public class TestController extends AbstractController {

    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    public void update(double pNow) {
        updateDrivetrain(pNow);
    }

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);

        rotate = Math.abs(rotate) > 0.01 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.01 ? throttle : 0.0; //Handling Deadband

        db.drivetrain.set(SHOULD_HOLD_POSITION, (throttle == 0.0 && rotate == 0.0) ? 1.0 : 0.0);

        if (throttle == 0.0 && rotate == 0.0) {
            db.drivetrain.set(DESIRED_THROTTLE, 0.0);
            db.drivetrain.set(DESIRED_TURN, 0.0);
        } else {
            db.drivetrain.set(SHOULD_HOLD_POSITION, 0.0);
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
            db.drivetrain.set(DESIRED_THROTTLE, -throttle);
            db.drivetrain.set(DESIRED_TURN, rotate);
        }

    }

}
