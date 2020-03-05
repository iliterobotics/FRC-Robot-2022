package us.ilite.robot.commands;

public interface IAutoCommand {
    void init(double pNow);
    boolean update(double pNow);
    void shutdown(double pNow);
    void updateDistance(double pCurrentDistance);
}
