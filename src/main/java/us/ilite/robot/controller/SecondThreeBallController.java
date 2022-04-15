package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class SecondThreeBallController extends BaseAutonController {
    private Timer mTimer;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(-14.5937), 2.0);
    //TODO actual distance from right up the hub to 1st ball is 118.75 inches, but that might be too much.
    // Play with the distance
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromInches(118.75));
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(104.5937), 2.0);
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromInches(117.470));
    //TODO play with the distances on the last turn and last leg
    private TurnToDegree mThirdTurn = new TurnToDegree(Rotation2d.fromDegrees(-120), 2.0);
    private DriveStraight mThirdLeg = new DriveStraight(Distance.fromInches(-120));
    private double mFirstTurnTime = 2.0,
                   mFirstLegTime = mFirstTurnTime + 4.0,
                   mSecondTurnTime = mFirstLegTime + 2.0,
                   mSecondLegTime = mSecondTurnTime + 4.0,
                   mThirdTurnTime = mSecondLegTime + 2.0,
                   mThirdLegTime = mThirdTurnTime + 4.0;
    private boolean fire = false;
    private boolean mFirstTurnDone = false;
    private boolean mFirstLegDone = false;
    private boolean mSecondTurnDone = false;
    private boolean mSecondLegDone = false;
    private boolean mThirdTurnDone = false;
    private boolean mThirdLegDone = false;
    public SecondThreeBallController() {
        mTimer = new Timer();
    }
    public void initialize() {
        mTimer.reset();
        mTimer.start();
    }
    public void updateImpl() {
        double time = mTimer.get();
        if (mTimer.get() < 0.5) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            mFirstTurn.init(time);
            fire = true;
        } else if (time < mFirstTurnTime) {
            fire = false;
            mFirstTurnDone = mFirstTurn.update(time) || time > mFirstTurnTime;
            if (mFirstTurnDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else if (time < mFirstTurnTime + 0.1) {
            mFirstLeg.init(time);
        } else if (time < mFirstLegTime) {
            mFirstLegDone = mFirstLeg.update(time) || time > mFirstLegTime;
            if (mFirstLegDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else if (time < mFirstLegTime + 0.1) {
            mSecondTurn.init(time);
        } else if (time < mSecondTurnTime) {
            mSecondTurnDone = mSecondTurn.update(time) || time > mSecondTurnTime;
            if (mSecondTurnDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else if (time < mSecondTurnTime + 0.1) {
            mSecondLeg.init(time);
        } else if (time < mSecondLegTime) {
            mSecondLegDone = mSecondLeg.update(time) || time > mSecondLegTime;
            if (mSecondLegDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else if (time < mSecondLegTime + 0.1) {
            mThirdTurn.init(time);
        } else if (time < mThirdTurnTime) {
            mThirdTurnDone = mThirdTurn.update(time) || time > mThirdTurnTime;
            if (mThirdTurnDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else if (time < mThirdTurnTime + 0.1) {
            mThirdLeg.init(time);
        } else if (time < mThirdLegTime) {
            mThirdLegDone = mThirdLeg.update(time) || time > mThirdLegTime;
            if (mThirdTurnDone) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        } else {
            fire = true;
        }
        if (fire) {
            fireCargo();
        } else {
            intakeCargo();
        }
    }
}
