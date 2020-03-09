package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.commands.CommandQueue;
import us.ilite.robot.commands.ICommand;

/**
 * Provides a wrapper for CommandQueue that allows commands to be stopped and started at will.
 */
public class CommandManager extends Module {

    private ILog mLog = Logger.createLog(CommandManager.class);

    private String mManagerTag = "";

    private CommandQueue desiredCommandQueue;
    private boolean lastRunCommandQueue;
    private boolean runCommandQueue;

    public CommandManager() {
        this.desiredCommandQueue = new CommandQueue();
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        runCommandQueue = lastRunCommandQueue = false;
    }

    @Override
    public void readInputs() {

    }

    @Override
    public void setOutputs() {
        updateCommands();
    }

    private void updateCommands() {

        // Don't initialize and update on same cycle
        if (shouldInitializeCommandQueue()) {
            mLog.warn(mManagerTag, ": Initializing command queue");
            desiredCommandQueue.init(clock.now());
        } else if(isRunningCommands()) {
            desiredCommandQueue.update(clock.now());
        }

        // Only check if we're done with queue if we're actually running...otherwise we're just spamming stopRunningCommands()
        if(isRunningCommands() && desiredCommandQueue.isDone()) {
            mLog.warn(mManagerTag, ": Command queue has completed execution");
            stopRunningCommands();
        }

        lastRunCommandQueue = runCommandQueue;
    }

    @Override
    public void shutdown() {

    }

    public boolean isRunningCommands() {
        return runCommandQueue;
    }

    /**
     * If we weren't running commands last cycle, initialize.
     */
    public boolean shouldInitializeCommandQueue() {
        return lastRunCommandQueue == false && runCommandQueue == true;
    }

    public CommandQueue getDesiredCommandQueue() {
        return desiredCommandQueue;
    }

    public void startCommands(ICommand ... pCommands) {
        // Only update the command queue if commands aren't already running
        if(!isRunningCommands()) {
            mLog.warn(mManagerTag, ": Starting command queue with a size of ", pCommands.length);
            runCommandQueue = true;
            desiredCommandQueue.setCommands(pCommands);
        } else {
            mLog.warn(mManagerTag, ": Set commands was called, but superstructure is already running commands");
        }
    }

    public void stopRunningCommands() {
        mLog.warn(mManagerTag, ": Stopping command queue");
        runCommandQueue = false;
        desiredCommandQueue.shutdown(clock.now());
        desiredCommandQueue.clear();
    }

    public CommandManager setManagerTag(String pManagerTag) {
        mManagerTag = pManagerTag;
        return this;
    }

}
