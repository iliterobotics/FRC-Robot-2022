package us.ilite.robot.controller;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

public class ThreeBallController extends BaseAutonController{
    public Timer mTimer;
    public void initialize(Trajectory pTrajectory) {
        //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }
    public void updateImpl() {
        fireCargo();
        if (mTimer.get() < 0.5) {
            db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            db.intake.set(EIntakeData.DESIRED_ROLLER_pct, 1.0);
            indexCargo();
        }
        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.POSITION);
        db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5.25);
        db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 5.25);

    }
}
