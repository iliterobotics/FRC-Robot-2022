package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.trajectory.Trajectory;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;

public class ShootMoveController extends BaseAutonController{

    public void initialize(Trajectory pTrajectory) {
        super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
    }
    public void updateImpl() {
        db.drivetrain.set(EDriveData.NEUTRAL_MODE, NeutralMode.Brake);
        if (mFirstLeg.get() < 0.5) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
            db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (mFirstLeg.get() == 0.5) {
            mTimer.reset();
        } else {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_pct, 1.0);
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.POSITION);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 10);
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 10);
            if (db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT) > 10) {
                if (db.drivetrain.get(EDriveData.ACTUAL_HEADING_DEGREES) >= 85 && db.drivetrain.get(EDriveData.ACTUAL_HEADING_DEGREES) <= 95) {
                    mCycleCount++;
                }
                if (mCycleCount >= 5) {
//                    initialize(TrajectoryCommandUtils.getOtherJSONTrajectory());
//                    execute();
                } else {
                    db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
                    db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
                    db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
                }
            }

        }
    }
}
