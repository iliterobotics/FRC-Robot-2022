package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.ELimelightData;
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
        super.updateDrivetrain();
        super.updateBalls();
        updateIntake();
        updateRollers();
        updateFeeder();
        moveFiveFeet();
    }

    private void updateIntake() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
            db.intake.set(ARM_STATE, Enums.EArmState.DEFAULT);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
            db.intake.set(ARM_STATE, Enums.EArmState.RETRACT);
        }
    }

    private void updateRollers() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_ROLLERS)) {
            db.intake.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.intake.set(DESIRED_pct, 1.0);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS)) {
            db.intake.set(ROLLER_STATE, Enums.EIntakeState.PERCENT_OUTPUT);
            db.intake.set(DESIRED_pct, -1.0);
        }

        if(db.driverinput.isSet(ELogitech310.A_BTN)) {
            db.cargo.set(SET_ROLLER_VEL_ft_s, 1000);
        }
    }
    private void updateLimelightTargetLock() {
        if (db.driverinput.isSet(InputMap.DRIVER.DRIVER_LIMELIGHT_LOCK_TARGET)) {
            db.limelight.set(ELimelightData.PIPELINE, Field2022.FieldElement.HUB_UPPER.pipeline());
            db.limelight.set(ELimelightData.LED_MODE, 1);
        } else {
            db.limelight.set(ELimelightData.PIPELINE, Field2022.FieldElement.NONE.pipeline());
            db.limelight.set(ELimelightData.LED_MODE, 0);
        }
    }

    private void updateFeeder() {
        if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, 1.0);
        } else if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_FEEDER)) {
            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
            db.feeder.set(EFeederData.SET_FEEDER_pct, -1.0);
        }
    }
}
