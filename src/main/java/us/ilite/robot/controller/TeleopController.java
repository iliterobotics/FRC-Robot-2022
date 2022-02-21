package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;


public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private static TeleopController INSTANCE;


    public static TeleopController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
        db.registerAllWithShuffleboard();
    }

    @Override
    protected void updateImpl() {
        db.registerAllWithShuffleboard();
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        super.updateDrivetrain(false);
        super.updateBalls();
        updateIntake();
        updateRollers();
        updateFeeder();
    }

    private void updateIntake() {
        if (db.operatorinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS)) { //left trigger
            db.cargo.set(ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (db.operatorinput.isSet(ELogitech310.RIGHT_TRIGGER_AXIS)) {
            db.cargo.set(ARM_STATE, Enums.EArmState.RETRACT);
        }
    }

    private void updateRollers() {
        if (db.operatorinput.isSet(ELogitech310.A_BTN)) {
            db.cargo.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.cargo.set(DESIRED_pct, 1.0);
        } else if (db.operatorinput.isSet(ELogitech310.B_BTN)) {
            db.cargo.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.cargo.set(DESIRED_pct, -1.0);
        }
    }

    private void updateFeeder() {
        if (db.operatorinput.isSet(ELogitech310.X_BTN)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0.5);
        } else if (db.operatorinput.isSet(ELogitech310.Y_BTN)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, -0.5);
        }
    }
}
