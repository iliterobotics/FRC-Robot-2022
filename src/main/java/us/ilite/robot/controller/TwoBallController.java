package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class TwoBallController extends BaseAutonController {
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromFeet(3.5));
    private Boolean mFirstLegComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromFeet(-3.5));
    private boolean mSecondLegComplete = false;
    private Timer mTimer;
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
            kSecondLegTimeEnd = kFirstTurnTimeEnd + 3.0;
    public void updateImpl() {
        double time = mTimer.get();
        if (time < 0.5) {
            setIntakeArmEnabled(true);
        }
        if (!mFirstLegComplete || (time > 1 && time < 3)) {
            intakeCargo();
            mFirstLegComplete = mFirstLeg.update(mTimer.get());
        }
        SmartDashboard.putBoolean("First leg complete", mFirstLegComplete);
        if (mFirstLegComplete || (time > 4 && time < 6)) {
            indexCargo();
            mSecondLeg.update(mTimer.get());
            setIntakeArmEnabled(false);
        }
        if (time > 8) {
            fireCargo();
        }
    }
}
