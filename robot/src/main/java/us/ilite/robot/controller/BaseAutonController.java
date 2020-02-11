package us.ilite.robot.controller;

import com.team2363.commands.HelixFollower;
import com.team2363.controller.PIDController;
import com.team319.trajectory.Path;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.modules.EDriveState;

public class BaseAutonController extends AbstractController {

    protected Path mActivePath = null;
    protected double mPathStartTime = 0d;
    private HelixFollowerImpl mPathFollower = null;
    private Velocities mVelocities = new Velocities();

    @Override
    protected void updateImpl(double pNow) {
        if (mVelocities.isFinished) {
            mPathFollower = null;
        }
        if(mPathFollower == null) {
            stopDrivetrain(pNow);
        } else {
            db.drivetrain.set(EDriveData.STATE, EDriveState.PATH_FOLLOWING_HELIX);
            db.drivetrain.set(EDriveData.L_PATH_FT_s, mVelocities.left);
            db.drivetrain.set(EDriveData.R_PATH_FT_s, mVelocities.right);
        }

    }

    protected void setActivePath(Path pPath) {
        mActivePath = pPath;
        mPathFollower = new HelixFollowerImpl(mActivePath, v -> {
           mVelocities.left = v.left;
           mVelocities.right = v.right;
           mVelocities.isFinished = v.isFinished;
        });
    }

    private class Velocities{
        public double left = 0;
        public double right = 0;
        public boolean isFinished = false;
    }

    private interface IHelixListener {
        void update(Velocities pUpdate);
    }

    private class HelixFollowerImpl extends HelixFollower {
        /** Used as a multi-threaded caching buffer */
        private Velocities mVelocities = new Velocities();
        private final IHelixListener mListener;
        private double mLastDistance = 0d;
        private double mLastHeading = 0d;

        /**
         * This will import the path class based on the name of the path provided
         *
         * @param path the name of the path to run
         */
        public HelixFollowerImpl(Path path, IHelixListener pListener) {
            super(path);
            mListener = pListener;
        }

        @Override
        public void resetDistance() {
            mVelocities.right = 0;
            mVelocities.left = 0;
            mVelocities.isFinished = false;
            mLastDistance = 0;
            db.drivetrain.set(EDriveData.STATE, EDriveState.RESET);
        }

        @Override
        public PIDController getHeadingController() {
            return new PIDController(0.1, 0.0, 0.0);
        }

        @Override
        public PIDController getDistanceController() {
            return new PIDController(0.1, 0.0, 0.0);
        }

        @Override
        public double getCurrentDistance() {
            // There is a small chance this fires after data.reset() but before the modules' readInput has run
            if(db.drivetrain.isSet(EDriveData.L_ACTUAL_POS_FT)) {
                mLastDistance = (db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT) +
                                db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT)
                ) / 2.0;
            }
            return mLastDistance;
        }

        @Override
        public double getCurrentHeading() {
            // There is a small chance this fires after data.reset() but before the modules' readInput has run
            if(db.imu.isSet(EGyro.HEADING_DEGREES)) {
                mLastHeading = db.imu.get(EGyro.HEADING_DEGREES) * Math.PI / 180.0;
            }
            return mLastHeading;
        }

        @Override
        public void useOutputs(double left, double right) {
            // Re-use the object to prevent GC
            mVelocities.left = left;
            mVelocities.right = right;
            mVelocities.isFinished = isFinished();
            mListener.update(mVelocities);
        }
    }
}
