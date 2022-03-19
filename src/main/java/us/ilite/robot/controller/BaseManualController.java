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
import static us.ilite.common.types.drive.EDriveData.DESIRED_TURN_PCT;
import static us.ilite.robot.Enums.EDriveState.POSITION;

/**
 * This class is a wrapper for functions/methods that are common between the Test and Teleop controllers
 */
public abstract class BaseManualController extends AbstractController {

    protected static final double DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;

    private final double kThrottleTransitionThreshold = 0.4;
    private final double kThrottleTransitionVelocity = 0.2;


    public void updateDrivetrain() {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS) * 0.75;

//        rotate = EInputScale.SIN_WITH_FGAIN1.map(rotate, 2);
        rotate = Math.abs(rotate) > 0.02 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.02 ? throttle : 0.0; //Handling Deadband

        if (db.driverinput.isSet(DRIVER_LIMELIGHT_LOCK_TARGET)) {
            db.drivetrain.set(STATE, EDriveState.TARGET_ANGLE_LOCK);
        } else {
            db.drivetrain.set(STATE, EDriveState.VELOCITY);
            if (throttle == 0.0 && rotate != 0.0) {
                throttle += 0.01;
            }
            DriveMessage d = new DriveMessage().throttle(throttle).turn(rotate).normalize();
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