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
    private boolean isBallAdded = false;
    private boolean isBallOut = false;
    private int numBalls = 0;

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
      //  clock.report("updateCargo", t -> updateCargo());
      //  clock.report("updateFeeder", t -> updateFeeder());
//        clock.report("updateIntake", t -> updateIntake());
        super.updateDrivetrain(false);
        updateIntakeOpenLoop();
        updateRollersOpenLoop();
        updateFeederOpenLoop();
    }

    private void updateCargo() {
//        SmartDashboard.putBoolean("Button pressed: ", db.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS));
        //Indexing balls coming in
        if (db.feeder.get(EFeederData.ENTRY_BEAM) == 1d) {
            if (!isBallAdded) {
                numBalls++;
                isBallAdded = true;
            }
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
        } else if (isBallAdded && db.feeder.get(EFeederData.ENTRY_BEAM) == 0d) {
//            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.0);
            isBallAdded = false;
        } //Indexing balls coming out
        else if (db.feeder.get(EFeederData.EXIT_BEAM) == 1d) {
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
            if (!isBallOut) {
                numBalls--;
                isBallOut = true;
            }
        } else if (isBallOut && db.feeder.get(EFeederData.EXIT_BEAM) == 0d) {
//            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.0);
            isBallOut = false;
        }
        db.feeder.set(EFeederData.NUM_BALLS, numBalls);
    }

    private void updateIntakeOpenLoop() {
        if (db.operatorinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS)) { //left trigger
            db.cargo.set(ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (db.operatorinput.isSet(ELogitech310.RIGHT_TRIGGER_AXIS)) {
            db.cargo.set(ARM_STATE, Enums.EArmState.RETRACT);
        }
    }
    private void updateRollersOpenLoop() {
        if (db.operatorinput.isSet(ELogitech310.A_BTN)) {
            db.cargo.set(STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.cargo.set(DESIRED_PCT, 1.0);
        } else if (db.operatorinput.isSet(ELogitech310.B_BTN)) {
            db.cargo.set(STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.cargo.set(DESIRED_PCT, -1.0);
        }
    }

    private void updateFeederOpenLoop() {
        if (db.operatorinput.isSet(ELogitech310.X_BTN)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 1.0);
        } else if (db.operatorinput.isSet(ELogitech310.Y_BTN)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, -1.0);
        }
    }

    private void updateFeeder() {
        //Shoot balls based off how many balls are in robot
        if (db.driverinput.isSet(ELogitech310.Y_BTN)) {
            if (db.feeder.get(EFeederData.NUM_BALLS) > 0) {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
            }
        }
        //todo - turn on some led when done
    }
}
