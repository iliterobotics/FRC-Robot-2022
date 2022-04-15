package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;
import us.ilite.robot.modules.NeoDriveModule;

public class ThreeBallController extends BaseAutonController {
    //Note that positive is clockwise and that negative is counter clockwise
    public Timer mTimer;
    //Old - 5.6
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromFeet(5.6));
    private boolean mFirstLegComplete = false;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(105d), 2d);
    private boolean mFirstTurnComplete = false;
    //Old - 7.7
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromFeet(7.7));
    private boolean mSecondLegComplete = false;
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(-60), 2d);
    private boolean mSecondTurnComplete = false;
    //Old - -6.4
    private DriveStraight mThirdLeg = new DriveStraight(Distance.fromFeet(-6.4));
    private boolean mThirdLegComplete = false;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mFirstLeg.init(mTimer.get());
    }

    public static double
        kFirstLegTimeEnd = 4.1,
        kFirstTurnTimeEnd = kFirstLegTimeEnd + 1.0,
        kSecondLegTimeEnd = kFirstTurnTimeEnd + 3.0,
        kSecondTurnEnd = kSecondLegTimeEnd + 2.0,
        kThirdLegTimeEnd = kSecondTurnEnd + 1.5;

    public void updateImpl() {
        double time = mTimer.get();
        boolean fire = false;
        if (time < 0.25) {
            fire = true;
        } else if (time < 0.5) {
            fire = false;
        } else if (time < kFirstLegTimeEnd) {
            mFirstLegComplete = mFirstLeg.update(mTimer.get());
        } else if (time < kFirstLegTimeEnd + 0.1 || mFirstLegComplete) {
            mFirstTurn.init(mTimer.get());
        } else if (time < kFirstTurnTimeEnd || mFirstLegComplete) {
            mFirstTurnComplete = mFirstTurn.update(mTimer.get());
        } else if (time < kFirstTurnTimeEnd + 0.1 || (mFirstLegComplete && mFirstTurnComplete)) {
            mSecondLeg.init(mTimer.get());
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
        } else if (time < kSecondLegTimeEnd || (mFirstLegComplete && mFirstTurnComplete)) {
            mSecondLegComplete = mSecondLeg.update(mTimer.get());
        } else if (time < kSecondLegTimeEnd + 0.1 || (mSecondLegComplete)) {
            mSecondTurn.init(mTimer.get());
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
        } else if (time < kSecondTurnEnd || (mSecondLegComplete)) {
            mSecondTurnComplete = mSecondTurn.update(mTimer.get());
        } else if (time < kSecondTurnEnd + 0.1 || mSecondTurnComplete) {
            mThirdLeg.init(mTimer.get());
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
        } else if (time < kThirdLegTimeEnd || (mSecondTurnComplete)) {
            mThirdLegComplete = mThirdLeg.update(mTimer.get());
        } else if (time > kThirdLegTimeEnd + 0.1) {
            setIntakeArmEnabled(false);
            fire = true;
        }

        if (fire) {
            fireCargo();
        } else {
            intakeCargo();
            indexCargo();
        }

    }
}
