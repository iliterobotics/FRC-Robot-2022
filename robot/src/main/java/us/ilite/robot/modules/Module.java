package us.ilite.robot.modules;

import us.ilite.common.Data;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

/**
 * The Module class defines how code written to control a specific subsystem (shooter, elevator, arm, etc.).
 * It also contains optional design patterns to adhere to.
 * All methods are passed a time, which is expected to be consistent between all modules updated in the same [mode]Periodic() call.
 */
public abstract class Module implements IModule {

    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;

    /*
    Although the Clock class removes the need for the now parameter, we will keep it since it may be useful to have
    in order to simulate certain conditions or edge cases.
     */

    /**
     * Runs when we init a new robot mode, for example teleopInit() or autonomousInit()
     */
    @Override
    public void modeInit(EMatchMode pMode){

    }

    /**
     * Shutdown/Cleanup tasks are performed here.
     */
    @Override
    public void shutdown() {

    }

    /**
     * Runs a self-test routine on this module's hardware.
     */
    @Override
    public boolean checkModule() {

        return true;
    }

    /**
     * Zeroes sensors.
     */
    @Override
    public void zeroSensors() {
    }

}
