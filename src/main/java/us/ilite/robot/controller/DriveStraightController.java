package us.ilite.robot.controller;


import us.ilite.robot.Enums;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.robot.Enums.EDriveState.*;

// Moves the robot 6 ft forward and turns 2 ft
// TODO Convert ft into degrees for turn
public class DriveStraightController extends BaseAutonController {

    double mCyclesPastDistance = 0;

    @Override
    public void updateImpl() {
        double leftPosition = db.drivetrain.get(L_ACTUAL_VEL_FT_s);
        double rightPosition = db.drivetrain.get(R_ACTUAL_VEL_FT_s);

        db.drivetrain.set(STATE, PATH_FOLLOWING_BASIC);

        if ((leftPosition+rightPosition)/2 <= 6) {
            db.drivetrain.set(L_DESIRED_POS, 6);
            db.drivetrain.set(R_DESIRED_POS, 6);
        } else {
            mCyclesPastDistance++;
        }

        if (mCyclesPastDistance >= 50) {
            db.drivetrain.set(L_DESIRED_POS, 2);
            db.drivetrain.set(R_DESIRED_POS, -2);
        }
    }

}
