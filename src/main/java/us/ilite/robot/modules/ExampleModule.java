package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.EMatchMode;

public class ExampleModule extends Module {

    private static final double kDelaySeconds = 5.0;

    private ILog mLog = Logger.createLog(ExampleModule.class);

    private Timer mTimer = new Timer();
    private boolean mOn = false;

    @Override
    public void modeInit(EMatchMode pMode) {
        mLog.error("MODE INIT");

        mTimer.reset();
        mTimer.start();
    }

    @Override
    protected void setOutputs() {

//        if(mTimer.hasPeriodPassed(kDelaySeconds)) {
//            mOn = !mOn;
//            mTimer.reset();
//        }

        if(mOn) {
            mLog.error("ON");

        } else {
            mLog.error("OFF");
        }

    }

    @Override
    public void shutdown() {
         mTimer.stop();
    }

    @Override
    public boolean checkModule() {
        return false;
    }

}
