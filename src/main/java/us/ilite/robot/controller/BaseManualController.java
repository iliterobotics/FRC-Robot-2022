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

    private int mCyclesHolding = 0;
    double mCyclesPastDistance = 0;

    public void updateDrivetrain() {
        double throttle = db.driverinput.get(THROTTLE_AXIS);
        double rotate = db.driverinput.get(TURN_AXIS) * 0.75;

        double left = db.driverinput.get(ELogitech310.LEFT_Y_AXIS);
        double right = db.driverinput.get(ELogitech310.RIGHT_Y_AXIS);

        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate = Math.abs(rotate) > 0.05 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.1 ? throttle : 0.0; //Handling Deadband

        left = Math.abs(left) > 0.005 ? left : 0.0;
        right = Math.abs(right) > 0.005 ? right : 0.0;

        if (rotate == 0d && throttle == 0d) {
            mCyclesHolding++;
        } else {
            mCyclesHolding = 0;
        }

        if (left == 0 && right == 0) {
            mCyclesHolding++;
        } else {
            mCyclesHolding = 0;
        }

        if (mCyclesHolding > 60) {
            db.drivetrain.set(STATE, EDriveState.HOLD);
            db.drivetrain.set(DESIRED_LEFT_PCT, left);
            db.drivetrain.set(DESIRED_RIGHT_PCT, right);
        }

        if (db.driverinput.isSet(DRIVER_LIMELIGHT_LOCK_TARGET)) {
           // db.drivetrain.set(STATE, EDriveState.TARGET_ANGLE_LOCK);
        } else if (db.driverinput.isSet(HOME_TO_DRIVER_STATION)) {
            db.drivetrain.set(STATE, EDriveState.HOME);
            throttle = 0;
            rotate = 0;
        } else if(mCyclesHolding > 60) {
            db.drivetrain.set(STATE, EDriveState.HOLD);
            db.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
            db.drivetrain.set(DESIRED_TURN_PCT, rotate);
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

            //TODO - Button here is bound to change once everything is integrated
            if (!db.driverinput.isSet(DRIVER_LIMELIGHT_LOCK_TARGET)) {
                db.drivetrain.set(STATE, EDriveState.VELOCITY);
                db.drivetrain.set(DESIRED_TURN_PCT, rotate);
            }
            db.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
            db.drivetrain.set(DESIRED_TURN_PCT, rotate);
        }

    }
    public void moveFiveFeet() {
        if (db.driverinput.isSet(ELogitech310.A_BTN)) {
            double leftPosition = db.drivetrain.get(L_ACTUAL_POS_FT);
            double rightPosition = db.drivetrain.get(R_ACTUAL_POS_FT);

            db.drivetrain.set(STATE, POSITION);

            if ((leftPosition+rightPosition)/2 <= 6) {
                db.drivetrain.set(L_DESIRED_POS_FT, 3);
                db.drivetrain.set(R_DESIRED_POS_FT, 3);
                System.out.println("reached");
            } else {
                mCyclesPastDistance++;
            }

            if (mCyclesPastDistance >= 50) {
                System.out.println("Desired distance reached, and it is currently " + mCyclesPastDistance + " cycles past the setpoint");
            }
        }
    }
}