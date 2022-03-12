package us.ilite.robot.modules;

public class ExceptionModule extends Module{

    @Override
    protected void readInputs() {
        throw new RuntimeException("This is only a test");
    }

    @Override
    protected void setOutputs() {
        throw new RuntimeException("This is just a test");
    }
}
