package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;

import java.util.LinkedList;
import java.util.List;

public class ModuleList extends Module {

    ILog mLogger = Logger.createLog(ModuleList.class);

    protected List<Module> mModules = new LinkedList<>();

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        mModules.forEach(module -> module.modeInit(pMode, pNow));
    }

    @Override
    public void readInputs(double pNow) {
        if(Robot.mode() == EMatchMode.TEST) {
            mModules.forEach(module -> Robot.CLOCK.report("R-"+module.getClass().getSimpleName(), t->module.readInputs(pNow)));
        } else {
            mModules.forEach(module -> module.readInputs(pNow));
        }

    }

    @Override
    public void setOutputs(double pNow) {
        if(Robot.mode() == EMatchMode.TEST) {
            mModules.forEach(module -> Robot.CLOCK.report("R-"+module.getClass().getSimpleName(), t->module.setOutputs(pNow)));
        } else {
            mModules.forEach(module -> module.setOutputs(pNow));
        }
    }

    @Override
    public void shutdown(double pNow) {
        mModules.forEach(module -> module.shutdown(pNow));
    }

    @Override
    public boolean checkModule(double pNow) {
        boolean allSuccessful = true;
            for (Module module : mModules) {
                boolean moduleSuccessful = module.checkModule(pNow);
            allSuccessful = allSuccessful && moduleSuccessful;
            if (!moduleSuccessful) {
                mLogger.error("Self-check failure for module: ", module.getClass());
            } else {
                mLogger.warn("Self-check success for module: ", module.getClass());
            }
        }

        return allSuccessful;
    }

    public void clearModules() {
        mModules.clear();
    }

    public void addModule(Module pModule) {
        mModules.add(pModule);
    }

}
