package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.DigitalBeamSensor;

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
        clock.report("updateCargo", t -> updateCargo());
        clock.report("updateFeeder", t -> updateFeeder());
        clock.report("updateIntake", t -> updateIntake());
        super.updateDrivetrain(false);
    }
    private void updateCargo() {
//        SmartDashboard.putBoolean("Button pressed: ", db.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS));
        //Indexing balls coming in
        if (db.feeder.get(EFeederData.ENTRY_BEAM) == 1d) {
            if(!isBallAdded) {
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
            if(!isBallOut) {
                numBalls--;
                isBallOut = true;
            }
        } else if (isBallOut && db.feeder.get(EFeederData.EXIT_BEAM) == 0d) {
//            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.0);
            isBallOut = false;
        }
        db.feeder.set(EFeederData.NUM_BALLS, numBalls);
    }

    private void updateIntake() {
        //If not max balls and button down, bring arm down and start intaking
        if (db.driverinput.isSet(ELogitech310.LEFT_TRIGGER_AXIS)) { //left trigger
                db.cargo.set(REV_PNEUMATIC_STATE, 0d);
                db.cargo.set(FWD_PNEUMATIC_STATE, 1d);
            if(db.feeder.get(EFeederData.NUM_BALLS) < 2) {
                db.cargo.set(SET_ROLLER_VEL_ft_s, Math.max(db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s), db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)) + 1000);
            }
            //If beam breaker is broken, add one ball
        } else {
                db.cargo.set(FWD_PNEUMATIC_STATE, 0d);
                db.cargo.set(REV_PNEUMATIC_STATE, 1d);
        }

        //Reverse intake
        if (db.driverinput.isSet(ELogitech310.R_BTN)) { //r button
            if (db.feeder.get(EFeederData.NUM_BALLS) == 0d) {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0);
            } else {
                db.feeder.set(EFeederData.SET_CONVEYOR_pct, -0.2);
            }
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
