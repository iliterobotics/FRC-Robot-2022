package us.ilite.robot.controller;

import com.team2363.commands.HelixFollower;
import com.team319.trajectory.Path;

public class ShootIntakeController extends AbstractController {
    protected Path mActivePath = null;
    protected double mPathStartTime = 0d;
    private HelixFollower mPathFollower = null;
    @Override
    protected void updateImpl(double pNow) {

    }
}
