package us.ilite.robot.controller;

import com.team319.trajectory.Path;

public class BaseAutonController extends AbstractController {

    protected Path mActivePath = null;
    protected double mPathStartTime = 0d;

    @Override
    protected void updateImpl(double pNow) {

    }
}
