package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;

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
        if (mTimer.get() < 2) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.SMART_MOTION);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5);
        }

    }
}
