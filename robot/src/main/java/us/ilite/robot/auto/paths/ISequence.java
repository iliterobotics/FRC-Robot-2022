package us.ilite.robot.auto.paths;

public interface ISequence {
    boolean isFinished();

    boolean finishedWithStep();

    boolean updateSequence(double pNow, double pDistanceTraveled);
}
