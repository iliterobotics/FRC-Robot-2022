package us.ilite.robot.modules;

import us.ilite.common.Data;
import us.ilite.common.types.EMatchMode;
import us.ilite.logging.ELevel;
import us.ilite.logging.Logger;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

/**
 * The Module class defines how code written to control a specific subsystem (shooter, elevator, arm, etc.).
 * It also contains optional design patterns to adhere to.
 * All methods are passed a time, which is expected to be consistent between all modules updated in the same [mode]Periodic() call.
 */
public abstract class Module {
    private static Logger sLogger = new Logger(Module.class, ELevel.ERROR);

    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;

    /*
    Although the Clock class removes the need for the now parameter, we will keep it since it may be useful to have
    in order to simulate certain conditions or edge cases.
     */

    /**
     * Runs when we init a new robot mode, for example teleopInit() or autonomousInit()
     */
    public void modeInit(EMatchMode pMode){
    }

    /**
     * Method to call the readInputs method and wrap it in a try/catch in case there are any
     * exceptions
     */
    public final void safeReadInputs() {
        try {
            readInputs();
         } catch(Exception e) {
            sLogger.debug("Got an exception in safeReadInputs",e);
        }
    }
    /**
     * Method to call the setOutputs method and wrap it in a try/catch in case there are any
     * exceptions
     */
    public final void safeSetOutputs() {
        try {
            setOutputs();
        } catch(Exception e) {
            sLogger.debug("Got an exception in safeSetOutputs",e);
        }
    }


    /**
     * The module's update function. Runs every time [mode]Periodic() is called (Roughly ~50Hz), or in a loop running at a custom frequency.
     */
    protected void readInputs() {

    }

    /**
     * Optional design pattern to keep hardware outputs all in one place.
     */
    protected void setOutputs() {

    }

    /**
     * Shutdown/Cleanup tasks are performed here.
     */
    public void shutdown() {

    }

    /**
     * Runs a self-test routine on this module's hardware.
     */
    public boolean checkModule() {

        return true;
    }

    /**
     * Zeroes sensors.
     */
    public void zeroSensors() {
    }

}
