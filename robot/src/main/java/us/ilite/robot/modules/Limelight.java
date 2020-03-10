package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.Field2020;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.IFieldComponent;

import static us.ilite.common.Field2020.FieldElement.OUTER_GOAL_UPPER_CORNERS;
import static us.ilite.robot.Enums.*;
import us.ilite.robot.modules.targetData.ITargetDataProvider;
import us.ilite.robot.vision.CameraConfig;
import us.ilite.robot.vision.Ilite3DSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ilite.common.types.EVisionGoal2020.*;
import static us.ilite.robot.vision.CameraConfig.LIMELIGHT_V2_LOW_RES;
import static us.ilite.robot.vision.Ilite3DSolver.xSort;
import static us.ilite.robot.vision.Ilite3DSolver.ySort;

/**
 * A module for interfacing with the Goal Limelight
 */
public class Limelight extends Module implements ITargetDataProvider {
    // These are for V2
    public static double llFOVVertical = 49.7;
    public static double llFOVHorizontal = 59.6;

    public static final IFieldComponent NONE = new IFieldComponent() {
        public int id() {return -1;}
        public double height() {return 0;}
        public int pipeline() {return 0;}
        public String toString() { return "NONE"; }
    };

    public static double kHeightIn = 41.0; // This is approximately correct for all of the hood angles that matter
    private IFieldComponent mGoal = OUTER_GOAL_UPPER_CORNERS; // The controller updates this
    private final CameraConfig mLimelight;
    private final Ilite3DSolver mIlite3DSolverSolver;

    private final NetworkTable mTable;

    public Limelight(String pNetworkTableName) {
        mLimelight = LIMELIGHT_V2_LOW_RES.setAddress(pNetworkTableName).setLensHeight(kHeightIn);
        mIlite3DSolverSolver = new Ilite3DSolver(
                mLimelight,
                Distance.fromInches(OUTER_GOAL_UPPER_CORNERS.height()),
                Distance.fromInches(OUTER_GOAL_UPPER_CORNERS.width()),
                Distance.fromInches(29.25 - 8.755)
        );
        mTable = NetworkTableInstance.getDefault().getTable(pNetworkTableName);
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        db.goaltracking.set(TARGET_ID, NONE.id());
    }

    @Override
    public void readInputs() {
        boolean targetValid = mTable.getEntry("tv").getDouble(Double.NaN) > 0.0;
        db.goaltracking.set(TV, targetValid);
        if(mGoal.id() >= 0 && targetValid) {
            db.goaltracking.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
            db.goaltracking.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
            db.goaltracking.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
            db.goaltracking.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
            if(db.goaltracking.get(TARGET_ID) != NONE.id()) {
                // Old way of doing it
                db.goaltracking.set(TARGET_RANGE_in, calcTargetDistance(mGoal));

                // New way of doing it
                List<Translation2d> corners = getCorners();
                mIlite3DSolverSolver.updatePoseToGoal(corners);
                db.goaltracking.set(T3D_X_in, mIlite3DSolverSolver.x().inches());
                db.goaltracking.set(T3D_Y_in, mIlite3DSolverSolver.y().inches());
                db.goaltracking.set(T3D_AZIMUTH_deg, mIlite3DSolverSolver.azimuth().degrees());
                db.goaltracking.set(T3D_AZ_OFFSET_deg, mIlite3DSolverSolver.offsetAzimuth().degrees());
                db.goaltracking.set(T3D_GOAL_RANGE_in, mIlite3DSolverSolver.range().inches());
                db.goaltracking.set(T3D_LEFT_RANGE_in, mIlite3DSolverSolver.leftRange().inches());
                db.goaltracking.set(T3D_RIGHT_RANGE_in, mIlite3DSolverSolver.rightRange().inches());
            }
        }
    }

    @Override
    public void setOutputs() {
        setNetworkTableValue("ledMode", LED_MODE);
        setNetworkTableValue("camMode", CAM_MODE);
        setNetworkTableValue("snapshot", SNAPSHOT_MODE);
        setNetworkTableValue("stream", STREAM_MODE);

        mGoal = (db.goaltracking.isSet(TARGET_ID)) ? NONE : db.goaltracking.get(TARGET_ID, Field2020.FieldElement.class);
        db.goaltracking.set(PIPELINE, mGoal.pipeline());
        mTable.getEntry("pipeline").setNumber(mGoal.pipeline());
        mLimelight.setElevationAngle(db.flywheel.safeGet(EShooterSystemData.HOOD_ANGLE_deg, 0d));
//        db.limelight.set(ANGLE_FROM_HORIZON, db.flywheel.get(ANGLE_FROM_HORIZON)); //TODO Add angle functionality to flywheel module

        SmartDashboard.putBoolean("Valid Goal", db.goaltracking.isSet(TV));
        SmartDashboard.putNumber("LL Latency (ms)", db.goaltracking.get(TL));
    }

    /**
     * Utility method
     */
    private void setNetworkTableValue(String pEntry, EVisionGoal2020 pEnum) {
        if(db.goaltracking.isSet(pEnum)) {
            mTable.getEntry(pEntry).setNumber(db.goaltracking.get(pEnum));
        }
    }

    @Override
    public void shutdown() {
        db.goaltracking.set(EVisionGoal2020.PIPELINE, Limelight.NONE.id());
        // Force LED off
        mTable.getEntry("ledMode").setNumber(LimelightLedMode.LED_OFF.ordinal());
    }

    public String toString() {
        return db.goaltracking.toCSV();
    }

    @Override
    public RobotCodex<EVisionGoal2020> getTargetingData() {
        return db.goaltracking;
    }

    @Override
    public double getCameraHeightIn() {
        return kHeightIn;
    }

    @Override
    public double getCameraAngleDeg() {
        return mLimelight.elevation_deg();
    }

    @Override
    public double getCameraToBumperIn() {
        return 0d;
    }

    @Override
    public double ty() {
        return 0;
    }


    /**
     * Credit to FRC254, from their 2019 public code
     *
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private List<Translation2d> getCorners() {
        double[] xCorners = mTable.getEntry("tcornx").getDoubleArray(mZeroArray);
        double[] yCorners = mTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        boolean tv = mTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!tv ||
                Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
                || xCorners.length != 8 || yCorners.length != 8) {
            return null;
        }

        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }
        if(mGoal.id() == OUTER_GOAL_UPPER_CORNERS.id()) {
            return topCorners(corners);
        } else {
            return bottomCorners(corners);
        }
    }

    /**
     * Credit to FRC254, from their 2019 public code
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public List<Translation2d> topCorners(List<Translation2d> pBoundingBoxPoints) {

        pBoundingBoxPoints.sort(xSort);

        List<Translation2d> left = pBoundingBoxPoints.subList(0, 4);
        List<Translation2d> right = pBoundingBoxPoints.subList(4, 8);

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2d> leftTop = left.subList(0, 2);
        List<Translation2d> rightTop = right.subList(0, 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2d leftCorner = leftTop.get(0);
        Translation2d rightCorner = rightTop.get(1);

        return List.of(leftCorner, rightCorner);
    }

    /**
     * Credit to FRC254, from their 2019 public code
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    // TODO - this is untested
    public List<Translation2d> bottomCorners(List<Translation2d> pBoundingBoxPoints) {
        pBoundingBoxPoints.sort(xSort);

        List<Translation2d> left = pBoundingBoxPoints.subList(0, 4);
        List<Translation2d> right = pBoundingBoxPoints.subList(4, 8);

        left.sort(ySort);
        right.sort(ySort);

        left = left.subList(2,4);
        right = right.subList(2, 4);

        left.sort(xSort);
        right.sort(xSort);

        Translation2d leftCorner = left.get(0);
        Translation2d rightCorner = right.get(1);

        return List.of(leftCorner, rightCorner);
    }
}