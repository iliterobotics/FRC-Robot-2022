package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class FourBallController extends BaseAutonController {
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(-10), 2);
    private boolean mFirstTurnComplete = false;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromInches(50));
    private boolean mFirstLegComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromInches(-45));
    private boolean mSecondLegComplete = false;
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(-10), 2);
    private boolean mSecondTurnComplete = false;
    private DriveStraight mThirdLeg = new DriveStraight(Distance.fromInches(120));
    private boolean mThirdLegComplete = false;
    private DriveStraight mFourthLeg = new DriveStraight(Distance.fromInches(-120));
    private boolean mFourthLegComplete = false;
    private Timer mTimer;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }

    private static double
            kFirstTurnTimeEnd =  1.5,
            kFirstLegTimeEnd = kFirstTurnTimeEnd + 2.0,
            kSecondLegTimeEnd = kFirstLegTimeEnd + 1.5,
            kSecondTurnTimeEnd = kFirstTurnTimeEnd + 1.5,
            kThirdLegTimeEnd = kSecondTurnTimeEnd + 4.0,
            kFourthLegTimeEnd = kThirdLegTimeEnd + 4.0;


    public void updateImpl() {
        double time = mTimer.get();
        if (time < 0.5) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            setIntakeArmEnabled(true);
            mFirstTurn.init(time);
            SmartDashboard.putString("Auton State", "Intake Out");
        }
        else if (time < kFirstTurnTimeEnd) {
            intakeCargo();
            mFirstTurnComplete = mFirstTurn.update(time) || time > kFirstTurnTimeEnd;
            if (mFirstTurnComplete) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
            SmartDashboard.putString("Auton State", "First turn " + mFirstTurnComplete);
        }
        else if (time < kFirstTurnTimeEnd + 0.1) {
            intakeCargo();
            mFirstLeg.init(time);
        }
        else if (time < kFirstLegTimeEnd) {
            intakeCargo();
            SmartDashboard.putString("Auton State", "First Leg " + mFirstLegComplete);
            mFirstLegComplete = mFirstLeg.update(time) || time > kFirstLegTimeEnd;
            if (mFirstTurnComplete) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        }
        else if (time < kFirstLegTimeEnd + 0.1) {
            mSecondLeg.init(time);
            intakeCargo();
        }
        else if (time < kSecondLegTimeEnd) {
            intakeCargo();
            stageBalls();
            SmartDashboard.putString("Auton State", "Second Leg " + mSecondLegComplete);
            mSecondLegComplete = mSecondLeg.update(time) || time > kSecondLegTimeEnd;
            if (mSecondLegComplete) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        }
        else if (time < kSecondLegTimeEnd + 0.1) {
            mSecondTurn.init(time);
        }
        else if (time < kSecondTurnTimeEnd) {
            SmartDashboard.putString("Auton State", "Second Turn " + mSecondLegComplete);
            mSecondTurnComplete = mSecondTurn.update(time) || time > kSecondTurnTimeEnd;
        }
        else if (time < kSecondTurnTimeEnd + 0.5){
            fireCargo();
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
        }
        else if (time < kSecondTurnTimeEnd + 0.6) {
            mThirdLeg.init(time);
        }
        else if (time < kThirdLegTimeEnd) {
            intakeCargo();
            mThirdLegComplete = mThirdLeg.update(time) || time < kThirdLegTimeEnd;
            SmartDashboard.putString("Auton State", "Third Leg " + mThirdLegComplete);
            if (mThirdLegComplete) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        }
        else if (time < kThirdLegTimeEnd + 0.1) {
           mFourthLeg.init(time);
        }
        else if (time < kFourthLegTimeEnd) {
            intakeCargo();
            mFourthLegComplete = mFourthLeg.update(time) || time < kFourthLegTimeEnd;
            SmartDashboard.putString("Auton State", "Fourth Leg " + mFourthLegComplete);
            if (mFourthLegComplete) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            }
        }
        else {
            fireCargo();
        }

    }
}
