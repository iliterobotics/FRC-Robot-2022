package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.TurnToDegree;

public class TexasSwitchController extends BaseAutonController {
    private Timer mTimer;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(90), 2d);
    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }
    public void updateImpl() {
        if (mTimer.get() < 0.5) {
            setIntakeArmEnabled(true);
        } else if (mTimer.get() < 1.0) {
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
            mFirstTurn.init(mTimer.get());
        }
        else if (mTimer.get() < 3.0) {
            db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
            reverseCargo();
        } else {

        }
    }
}
