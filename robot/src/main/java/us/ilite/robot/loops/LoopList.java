package us.ilite.robot.loops;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EMatchMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LoopList extends Loop {

    ILog mLogger = Logger.createLog(LoopList.class);

    protected List<Loop> mLoops = new ArrayList<>();

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        mLoops.forEach(module -> module.modeInit(pMode, pNow));
    }

    @Override
    public void readInputs(double pNow) {
        mLoops.forEach(module -> module.readInputs(pNow));
    }

    @Override
    public void setOutputs(double pNow) {
        mLoops.forEach(module -> module.setOutputs(pNow));
    }

    @Override
    public void shutdown(double pNow) {
        mLoops.forEach(module -> module.shutdown(pNow));
    }

    @Override
    public boolean checkModule(double pNow) {
        boolean allSuccessful = true;
        for (Loop loop : mLoops) {
            boolean moduleSuccessful = loop.checkModule(pNow);
            allSuccessful = allSuccessful && moduleSuccessful;
            if (!moduleSuccessful) {
                mLogger.error("Self-check failure for module: ", loop.getClass());
            } else {
                mLogger.warn("Self-check success for module: ", loop.getClass());
            }
        }

        return allSuccessful;
    }

    @Override
    public void loop(double pNow) {
        mLoops.forEach(loop -> loop.loop(pNow));
    }

    public void setLoops(Loop ... pLoops) {
        mLoops.clear();
        mLoops.addAll(Arrays.asList(pLoops));
    }

}
