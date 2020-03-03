package us.ilite.robot.controller;

import us.ilite.common.config.Settings;
import us.ilite.common.types.input.EInputScale;
import us.ilite.robot.modules.DriveMessage;
import static us.ilite.robot.Enums.*;

import static us.ilite.common.config.InputMap.DRIVER.*;
import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.drive.EDriveData.DESIRED_TURN_PCT;

/**
 * This class is a wrapper for functions/methods that are common between the Test and Teleop controllers
 */
public abstract class BaseManualController extends AbstractController {

    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    void updateDrivetrain(double pNow) {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS) * 0.75;
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate = Math.abs(rotate) > 0.01 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.01 ? throttle : 0.0; //Handling Deadband

        if(throttle == 0.0 && rotate == 0.0) {
            db.drivetrain.set(STATE, EDriveState.HOLD);
            db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
            db.drivetrain.set(DESIRED_TURN_PCT, 0.0);
        } else {
            db.drivetrain.set(STATE, EDriveState.VELOCITY);
            if (throttle == 0.0 && rotate != 0.0) {
                throttle += 0.01;
            }
            DriveMessage  d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
            throttle = d.getThrottle();
            rotate = d.getTurn();
            if (db.driverinput.isSet(SNAIL_MODE) && db.driverinput.get(SNAIL_MODE) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
                throttle *= Settings.Input.kSnailModePercentThrottleReduction;
                rotate *= Settings.Input.kSnailModePercentRotateReduction;
            }

            db.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
            db.drivetrain.set(DESIRED_TURN_PCT, rotate);
        }

    }
}