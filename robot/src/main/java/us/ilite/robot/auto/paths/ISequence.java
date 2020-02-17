package us.ilite.robot.auto.paths;

public interface ISequence {
    boolean isFinished();

    private boolean finishedWithStep() {
        return false;
    }

    boolean updateSequence(double pNow);
}
