package us.ilite.robot.modules;

import us.ilite.common.types.EMatchMode;

public interface IModule {
    void modeInit(EMatchMode pMode);

    /**
     * The module's update function. Runs every time [mode]Periodic() is called (Roughly ~50Hz), or in a loop running at a custom frequency.
     */
    void readInputs();

    /**
     * Optional design pattern to keep hardware outputs all in one place.
     */
    void setOutputs();

    void shutdown();

    boolean checkModule();

    void zeroSensors();
}
