package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class ThreeBallController extends BaseAutonController{
    public Timer mTimer;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromFeet(5.6));
    private boolean mFirstLegComplete = false;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(110d), 2d);
    private boolean mFirstTurnComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromFeet(7.0));
    private boolean mSecondLegComplete = false;
    public void initialize(Trajectory pTrajectory) {
        //  super.initialize(TrajectoryCommandUtils.getJSONTrajectory());
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mFirstLeg.init(mTimer.get());
    }

    private static double
        kFirstLegTimeEnd = 4.0,
        kFirstTurnTimeEnd = kFirstLegTimeEnd + 3.0,
        kSecondLegTimeEnd = kFirstTurnTimeEnd + 3.0
                ;
    public void updateImpl() {
        double time = mTimer.get();
        boolean fire = false;
        if(time < 0.25) {
            fire = true;
        } else if (time < 0.5) {
            fire = false;
        } else if (time < kFirstLegTimeEnd) {
            mFirstLegComplete = mFirstLeg.update(mTimer.get());
        } else if (time < kFirstLegTimeEnd + 0.1 || mFirstLegComplete) {
            mFirstTurn.init(mTimer.get());
        }else if (time < kFirstTurnTimeEnd || mFirstLegComplete) {
            mFirstTurnComplete = mFirstTurn.update(mTimer.get());
        }else if (time < kFirstTurnTimeEnd + 0.1 || (mFirstLegComplete && mFirstTurnComplete)) {
            mSecondLeg.init(mTimer.get());
        }else if (time < kSecondLegTimeEnd || (mFirstLegComplete && mFirstTurnComplete)) {
            mSecondLegComplete = mSecondLeg.update(mTimer.get());
        } // re-line up and fire

        if(fire) {
            fireCargo();
        } else {
            intakeCargo();
            indexCargo();
        }

    }
}
