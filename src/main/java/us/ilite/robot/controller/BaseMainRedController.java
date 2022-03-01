package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;

public class BaseMainRedController extends BaseAutonController {
    
    private int mCycleCount = 0;

    @Override
    public void updateImpl() {
        db.drivetrain.set(EDriveData.NEUTRAL_MODE, NeutralMode.Brake);
        if (mFirstLeg.get() < 0.5) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
            db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (mFirstLeg.get() == 0.5) {
            mTimer.reset();
        } else {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_pct, 1.0);
            execute();
            if (isFinished()) {
                if (db.drivetrain.get(EDriveData.ACTUAL_HEADING_DEGREES) >= 85 && db.drivetrain.get(EDriveData.ACTUAL_HEADING_DEGREES) <= 95) {
                    mCycleCount++;
                }
                if (mCycleCount >= 5) {
                    initialize(TrajectoryCommandUtils.getOtherJSONTrajectory());
                    execute();
                } else {
                    db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
                    db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
                    db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
                }
            }

        }
    }
}
