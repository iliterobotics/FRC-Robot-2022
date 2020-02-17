package us.ilite.robot.controller;

import com.team2363.commands.IliteHelixFollower;
import com.team2363.controller.PIDController;
import com.team319.trajectory.Path;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.auto.paths.AutonSelection;
import us.ilite.robot.Robot;
import us.ilite.robot.auto.paths.BobUtils;
import us.ilite.robot.modules.EDriveState;

import java.util.Map;

public class BaseAutonController extends AbstractController {

    protected double mDelayCycleCount;
    protected Path mActivePath = null;
    protected double mPathStartTime = 0d;
    private HelixFollowerImpl mPathFollower = null;
    private final String kPathAssociation;

    public BaseAutonController(String pPathAssociation) {
        kPathAssociation = pPathAssociation;
        mDelayCycleCount = AutonSelection.mDelaySeconds;
        setActivePath(getPathsFromController().get((String) getPathsFromController().keySet().toArray()[AutonSelection.mPathNumber]));
    }

    public BaseAutonController() {
        this("DEFAULT");
    }

    @Override
    protected void updateImpl(double pNow) {
        if(mPathStartTime == 0) {
            mPathStartTime = pNow;
        }
        if (mPathFollower != null && mPathFollower.isFinished()) {
            mPathFollower = null;
        }
        if(mPathFollower == null) {
            stopDrivetrain(pNow);
        } else {
            mPathFollower.execute(pNow);
        }
    }

    protected void setActivePath(Path pPath) {
        mActivePath = pPath;
        mPathFollower = new HelixFollowerImpl(mActivePath);
        mPathFollower.initialize();
    }

    public Map<String, Path> getPathsFromController() {
        Map<String, Path> mAvailablePaths = BobUtils.getAvailablePaths();
        for (int i = 0; i < mAvailablePaths.entrySet().toArray().length - 1; i++) {
            Map.Entry<String, Path> entry = (Map.Entry<String, Path>) mAvailablePaths.entrySet().toArray()[i];
            if (!entry.getKey().toLowerCase().contains(kPathAssociation.toLowerCase())) {
                mAvailablePaths.remove(entry.getKey());
                i--;
            }
        }
        return mAvailablePaths;
    }

    private class HelixFollowerImpl extends IliteHelixFollower {
        /** Used as a multi-threaded caching buffer */
        private double mLastDistance = 0d;
        private double mLastHeading = 0d;
        private PIDController mDistanceController = new PIDController(
                1.5, 0.0, 0.0);
        private PIDController mHeadingController = new PIDController(
                0.001, 0.0, 0.0);
        private double mPathStartTime = 0.0;


        /**
         * This will import the path class based on the name of the path provided
         *
         * @param path the name of the path to run
         */
        public HelixFollowerImpl(Path path) {
            super(path);
        }

        @Override
        public void resetDistance() {
            mPathStartTime = Robot.CLOCK.getCurrentTime();
            mLastDistance = 0;
            db.drivetrain.set(EDriveData.STATE, EDriveState.RESET);
        }

        @Override
        public PIDController getHeadingController() {
            return mHeadingController;
        }

        @Override
        public PIDController getDistanceController() {
            return mDistanceController;
        }

        @Override
        public double getCurrentDistance() {
            // There is a small chance this fires after data.reset() but before the modules' readInput has run
//                mLastDistance = (db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT) +
//                                db.drivetrain.get(EDriveData.R_ACTUAL_POS_FT)
//                ) / 2.0;
            return db.drivetrain.get(EDriveData.L_ACTUAL_POS_FT);
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
            db.drivetrain.set(EDriveData.STATE, EDriveState.PATH_FOLLOWING_HELIX);
            db.drivetrain.set(EDriveData.L_PATH_FT_s, left);
            db.drivetrain.set(EDriveData.R_PATH_FT_s, right);
            db.drivetrain.set(EDriveData.PATH_ERR_ft, mDistanceController.getError());
        }

        protected void moveToNextSegment(double pNow) {
            currentSegment = BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime);
            if (currentSegment == -1)
            {
                isFinished = true;
            }
        }

        public void execute(double pNow) {
            super.execute();
            moveToNextSegment(pNow);
            super.calculateOutputs();
            if(isFinished()) {
                stopDrivetrain(0.0);
            }
        }
    }


}
