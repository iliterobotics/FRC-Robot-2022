package us.ilite.robot.controller;

import edu.wpi.first.math.trajectory.Trajectory;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.TrajectoryCommandUtils;

public class ShootIntakeMoveController extends BaseAutonController {
    private boolean mHasShotPreload;
    public ShootIntakeMoveController() {
        mHasShotPreload = false;
    }
    public void updateImpl() {
        if (!mHasShotPreload) {
            mHasShotPreload = true;
            //TODO implement firing sequence right here
            super.execute();
        }
    }
}
