package us.ilite.robot.modules;

import org.springframework.stereotype.Component;
import us.ilite.common.types.EMatchMode;

@Component
public class TestModule2 implements IModule{
    @Override
    public void modeInit(EMatchMode pMode) {

    }

    @Override
    public void readInputs() {

    }

    @Override
    public void setOutputs() {

    }

    @Override
    public void shutdown() {

    }

    @Override
    public boolean checkModule() {
        return false;
    }

    @Override
    public void zeroSensors() {

    }
}
