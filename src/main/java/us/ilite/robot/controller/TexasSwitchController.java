package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class TexasSwitchController extends BaseAutonController {
    private Timer mTimer;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(-45), 2d);
    private DriveStraight mTaxi = new DriveStraight(Distance.fromFeet(4.5));
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }
    public void updateImpl() {
        if (mTimer.get() < 1.0) {
            reverseCargo();
            setIntakeArmEnabled(true);
        } else if (mTimer.get() < 1.1) {
            mFirstTurn.init(mTimer.get());
        } else if (mTimer.get() < 3.0) {
            setIntakeArmEnabled(false);
            mFirstTurn.update(mTimer.get());
        } else if (mTimer.get() < 3.1) {
            mTaxi.init(mTimer.get());
        } else if (mTimer.get() < 6.0) {
            mTaxi.update(mTimer.get());
        }
    }
}
