package us.ilite.robot.controller;

import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.robot.Enums;

public class ReverseFeederIntakeController extends BaseAutonController {
    public void updateImpl() {
        db.feeder.set(EFeederData.SET_FEEDER_pct, -1.0);
        setIntakeArmEnabled(true);
        db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
        db.intake.set(EIntakeData.ROLLER_PCT, -1.0);
    }
}
