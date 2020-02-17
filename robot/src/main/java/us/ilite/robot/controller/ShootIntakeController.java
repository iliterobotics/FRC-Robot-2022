package us.ilite.robot.controller;

import com.team2363.commands.HelixFollower;
import com.team2363.commands.IliteHelixFollower;
import com.team2363.controller.PIDController;
import com.team2363.logger.HelixLogger;
import com.team319.trajectory.Path;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
import us.ilite.robot.auto.paths.BobUtils;
import us.ilite.robot.auto.paths.SimpleSequence;
import us.ilite.robot.modules.EDriveState;

public class ShootIntakeController extends BaseAutonController {
    protected Path mActivePath = null;
    protected double mPathStartTime = 0d;
    private double mPathDistance = 0d;
    private SimpleSequence mSimpleSequence = new SimpleSequence();
    @Override
    protected void updateImpl(double pNow) {
        super.updateImpl(pNow);
        mSimpleSequence.updateSequence(pNow, mPathDistance);
    }

}
