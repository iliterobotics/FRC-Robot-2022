package us.ilite.robot.controller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;

public class BaseMainRedController extends BaseAutonController {

    private Timer mTimer = new Timer();
    private boolean isFirstSequenceDone = false;
    private boolean isSecondSequenceDone = false;
    private boolean isThirdSequenceDone = false;
    private boolean isFourthSequenceDone = false;
    private boolean isFifthSequenceDone = false;

    @Override
    public void updateImpl() {
        //TODO tune position control and fix how far the drive goes on each leg along with desired angle
        db.drivetrain.set(EDriveData.NEUTRAL_MODE, NeutralMode.Brake);
        db.intake.set(EIntakeData.ARM_STATE, Enums.EArmState.DEFAULT);
        db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
        if (mTimer.get() < 0.5) {
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.POSITION);
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 5);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5);
            isFirstSequenceDone = true;
        }
        if (isFirstSequenceDone) {
            isFirstSequenceDone = false;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
            isSecondSequenceDone = true;
        }
        if (isSecondSequenceDone) {
            isSecondSequenceDone = false;
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 5);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5);
            isThirdSequenceDone = true;
        }
        if (isThirdSequenceDone) {
            isThirdSequenceDone = false;
            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.TURN_TO);
            db.drivetrain.set(EDriveData.DESIRED_TURN_ANGLE_deg, 120);
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.05);
            isFourthSequenceDone = true;
        }
        if (isFourthSequenceDone) {
            isFourthSequenceDone = false;
            db.drivetrain.set(EDriveData.R_DESIRED_POS_FT, 5);
            db.drivetrain.set(EDriveData.L_DESIRED_POS_FT, 5);
            isFifthSequenceDone = true;
        }
        if (isFifthSequenceDone) {
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
        }
    }
    public boolean isFinished() {
        return mTimer.get() >= 15;
    }
}
