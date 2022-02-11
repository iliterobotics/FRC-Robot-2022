package us.ilite.robot.commands;

@Deprecated
public interface IAutoCommand {
    void init(double pNow);
    boolean update(double pNow);
    void shutdown(double pNow);
    void updateDistance(double pCurrentDistance);
}
