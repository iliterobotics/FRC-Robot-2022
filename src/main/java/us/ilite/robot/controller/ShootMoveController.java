package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.commands.DriveStraight;

public class ShootMoveController extends BaseAutonController{

    public Timer mTimer;
    public Timer mSecondLeg;
    public boolean isFirstLeg = true;
    public boolean isSecondLeg = false;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromFeet(7));
    public void initialize(Trajectory pTrajectory) {
      //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mSecondLeg = new Timer();
    }
    public void updateImpl() {
        if (mTimer.get() < 0.5) {
            fireCargo();
            mFirstLeg.init(mTimer.get());
        }
        else if (mTimer.get() < 5) {
            mFirstLeg.update(mTimer.get());
        }

    }
}
