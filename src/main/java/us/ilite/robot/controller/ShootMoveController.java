package us.ilite.robot.controller;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

public class ShootMoveController extends BaseAutonController{

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
        if (mTimer.get() < 3.1 && mTimer.get() > 2.0) {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_ROLLER_pct, 1.0);
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
        }
        if (mTimer.get() > 3.1 && mTimer.get() < 5.0) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
        }

    }
}
