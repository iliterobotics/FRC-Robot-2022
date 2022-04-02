package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class TwoBallController extends BaseAutonController {
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(-10), 2);
    private boolean mFirstTurnComplete = false;
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromInches(50));
    private boolean mFirstLegComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromInches(-45));
    private boolean mSecondLegComplete = false;
    private TurnToDegree mSecondTurn = new TurnToDegree(Rotation2d.fromDegrees(-10), 2);
    private boolean mSecondTurnComplete = false;
    private Timer mTimer;
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }

    private static double
            kFirstTurnTimeEnd =  2.5,
            kFirstLegTimeEnd = kFirstTurnTimeEnd+ 4.0,
            kSecondLegTimeEnd = kFirstLegTimeEnd + 2.5,
            kSecondTurnTimeEnd = kFirstTurnTimeEnd + 2.5;


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
        }

    }
}
