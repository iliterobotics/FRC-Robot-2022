package us.ilite.robot.controller;

import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

public class BaseMainRedController extends BaseAutonController {

    private boolean hasMovedFirst = false;
    private boolean hasTurnedFirst = false;
    private boolean moveOnLastLeg = false;

    @Override
    public void updateImpl() {
        if (!hasMovedFirst) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.POSITION);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 6.33);
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 6.33);
            hasMovedFirst = true;
            hasTurnedFirst = true;
        }
        if (hasTurnedFirst) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_FOR);
            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 70);
            //TODO figure out how to switch between turn to and position
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 4.2);
            db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
            db.intake.set(EIntakeData.DESIRED_pct, 1.0);
            hasTurnedFirst = false;
            moveOnLastLeg = true;
        }
        if (moveOnLastLeg) {

        }
    }
}
