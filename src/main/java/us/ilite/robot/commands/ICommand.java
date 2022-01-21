package us.ilite.robot.commands;

@Deprecated
public interface ICommand {
	void init(double pNow);
	boolean update(double pNow);
	void shutdown(double pNow);
}
