package us.ilite.robot.controller;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.InputMap;
import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import static us.ilite.common.types.EIntakeData.*;


public class TeleopController extends BaseManualController { //copied from TestController, needs editing

    private ILog mLog = Logger.createLog(TeleopController.class);
    private static TeleopController INSTANCE;

    public static TeleopController getInstance() {
        if(INSTANCE == null) {
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
    }
}
