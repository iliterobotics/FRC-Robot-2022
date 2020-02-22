package us.ilite.robot.auto.paths;

public class YoinkSequence implements ISequence {
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean finishedWithStep() {
        return false;
    }

    @Override
    public boolean updateSequence(double pNow, double pDistanceTraveled) {
        return false;
    }
}
