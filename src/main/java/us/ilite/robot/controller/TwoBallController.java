package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class TwoBallController extends BaseAutonController {
    //This was -10 initially
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(-6), 2);
    private boolean mFirstTurnComplete = false;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromInches(56));
    private boolean mFirstLegComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromInches(-43));
    private boolean mSecondLegComplete = false;
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(-6), 2);
    private boolean mSecondTurnComplete = false;
    private DriveStraight mLeaveTarmac = new DriveStraight(Distance.fromFeet(7.5));
    private Timer mTimer;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }

    //First leg time was 3.0 seconds shaved to 1.5 (1.5 second taken off)
    private static double
            kFirstTurnTimeEnd = 1.0,
            kFirstLegTimeEnd = kFirstTurnTimeEnd + 1.5,
            kSecondLegTimeEnd = kFirstLegTimeEnd + 2.5,
            kSecondTurnTimeEnd = kSecondLegTimeEnd + 1.0;


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
            if(mFirstTurnComplete) {
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
            if(mFirstTurnComplete) {
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
            mSecondTurn.update(time);
        }
        else {
            fireCargo();
            setIntakeArmEnabled(false);
        }
//        else if (time < kSecondTurnTimeEnd + 1.1) {
//            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
//            db.feeder.set(EFeederData.SET_FEEDER_pct, 0.0);
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
//            mLeaveTarmac.init(time);
//        }
//        else {
//            mLeaveTarmac.update(time);
//        }

    }
}
