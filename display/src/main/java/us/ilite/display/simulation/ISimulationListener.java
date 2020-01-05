package us.ilite.display.simulation;


import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface ISimulationListener {

    void update(double pTimeStamp, Pose2d pCurrentPose);

}
