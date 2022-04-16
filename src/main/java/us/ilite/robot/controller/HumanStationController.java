package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class HumanStationController extends BaseAutonController {
    private Timer mTimer;
    private boolean fire =  false;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(43.0262), 2d);
    private boolean mFirstTurnDone = false;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromInches(124.046));
    private boolean mFirstLegDone = false;
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(15.315), 2d);
    private boolean mSecondTurnDone = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromInches(159.947));
    private boolean mSecondLegDone = false;
    private DriveStraight mThirdLeg = new DriveStraight(Distance.fromInches(-159.947));
    private boolean mThirdLegDone = false;
    private TurnToDegree mThirdTurn = new TurnToDegree(Rotation2d.fromDegrees(-15.315), 2d);
    private boolean mThirdTurnDone = false;
    private DriveStraight mFourthLeg = new DriveStraight(Distance.fromInches(-124.046));
    private boolean mFourthLegDone = false;
    private TurnToDegree mFourthTurn = new TurnToDegree(Rotation2d.fromDegrees(-43.0262), 2d);
    private boolean mFourthTurnDone = false;

    private double mFirstTurnTime = 2.0,
                    mFirstLegTime = mFirstTurnTime + 4.0,
                    mSecondTurnTime = mFirstLegTime + 2.0,
                    mSecondLegTime = mSecondTurnTime + 4.0,
                    mThirdLegTime = mSecondLegTime + 4.0,
                    mFourthTurnTime = mThirdLegTime + 2.0,
                    mFourthLegTime = mFourthTurnTime + 4.0,
                    mFifthTurnTime = mFourthLegTime + 2.0;

    public HumanStationController() {
        mTimer = new Timer();
    }
    public void initialize() {
        mTimer.reset();
        mTimer.start();
    }
    public void updateImpl() {
        double time = mTimer.get();
        if (time < 0.5) {
            fire = true;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            mFirstTurn.init(time);
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
        }
        if (fire) {
            fireCargo();
        } else {
            intakeCargo();
        }
    }
}
