package us.ilite.robot.controller;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

public class ThreeBallController extends BaseAutonController {
    public Timer mTimer;
    public Timer mSecondLeg;
    public boolean isFirstLeg = true;
    public boolean isSecondLeg = false;
    public void initialize(Trajectory pTrajectory) {
        //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mSecondLeg = new Timer();
    }
    public void updateImpl() {
        if (mTimer.get() < 2.0) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
            db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
        }
        if (mTimer.get() > 2.3 && mTimer.get() < 3.3) {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_pct, 1.0);
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.POSITION);
            double r_actual = db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT);
            double l_actual = db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT);
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, (l_actual + 6.0));
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, (r_actual + 6.0));
        }
        if (mTimer.get() > 3.6 && mTimer.get() < 5.0) {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_pct, 1.0);
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
        }
//        if (mTimer.get() > 5.3 && mTimer.get() < 6.0) {
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
//        }
//        if (mTimer.get() > 6.3 && mTimer.get() < 8.0) {
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
//            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 65);
//            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
//        }
//        if (mTimer.get() > 8.3 && mTimer.get() < 10.0) {
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
//        }
//        if (mTimer.get() > 10.3) {
//            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
//            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
//        }

    }
}
