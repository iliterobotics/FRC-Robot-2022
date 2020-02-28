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
        double rotate = -db.driverinput.get(TURN_AXIS);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate = Math.abs(rotate) > 0.01 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.01 ? throttle : 0.0; //Handling Deadband

        if (db.driverinput.isSet(DRIVER_LIMELIGHT_LOCK_TARGET)) {
            db.drivetrain.set(STATE, EDriveState.TARGET_ANGLE_LOCK);
        } else if(throttle == 0.0 && rotate == 0.0) {
            db.drivetrain.set(STATE, EDriveState.HOLD);
            db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
            db.drivetrain.set(DESIRED_TURN_PCT, 0.0);
        }
        
        if (throttle != 0.0 || rotate != 0.0) {
            if (throttle == 0.0 && rotate != 0.0) {
                throttle += 0.01;
            }
            var d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
            throttle = d.getThrottle();
            rotate = d.getTurn();
            if (db.driverinput.isSet(SNAIL_MODE) && db.driverinput.get(SNAIL_MODE) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
                throttle *= Settings.Input.kSnailModePercentThrottleReduction;
                rotate *= Settings.Input.kSnailModePercentRotateReduction;
            }

            //TODO - Button here is bound to change once everything is integrated
            if (!db.driverinput.isSet(DRIVER_LIMELIGHT_LOCK_TARGET)) {
                db.drivetrain.set(STATE, EDriveState.VELOCITY);
                db.drivetrain.set(DESIRED_TURN_PCT, rotate);
            }
            db.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);

        }

    }
}
