package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.EInputScale;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.modules.DriveMessage;
import static us.ilite.robot.Enums.*;

import static us.ilite.common.config.InputMap.DRIVER.*;
import static us.ilite.common.types.drive.EDriveData.*;

/**
 * This class is a wrapper for functions/methods that are common between the Test and Teleop controllers
 */
public abstract class BaseManualController extends AbstractController {

    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    private final double kThrottleTransitionThreshold = 0.4;
    private final double kThrottleTransitionVelocity = 0.2;


    public void updateDrivetrain() {
        db.drivetrain.set(STATE, EDriveState.VELOCITY);
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS) * 0.80;

//        throttle = EInputScale.EXPONENTIAL.map(throttle, 2);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);

        rotate = Math.abs(rotate) > 0.02 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.02 ? throttle : 0.0; //Handling Deadband

        if (db.driverinput.isSet(SNAIL_MODE) && db.driverinput.get(SNAIL_MODE) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
            throttle *= Settings.Input.kSnailModePercentThrottleReduction;
            rotate *= Settings.Input.kSnailModePercentRotateReduction;
        }

        if (db.driverinput.isSet(TARGET_LOCK)) {
            db.drivetrain.set(DESIRED_THROTTLE_PCT, Math.min(throttle, 0.75));
            db.drivetrain.set(DESIRED_TURN_PCT, rotate);
        } else {
            if (throttle == 0.0 && rotate != 0.0) {
                throttle += 0.01;
            }
            DriveMessage d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
            throttle = d.getThrottle();
            rotate = d.getTurn();
            db.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
            db.drivetrain.set(DESIRED_TURN_PCT, rotate);
        }

    }
}