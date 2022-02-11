package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.Field2022;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.IFieldComponent;

import static us.ilite.common.types.EVisionGoal2020.T3D_TOP_Y_in;
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
    @Override
    public void readInputs() {

    }

    @Override
    public void setOutputs() {

    }

    @Override
    public RobotCodex<EVisionGoal2020> getTargetingData() {
        return null;
    }

    @Override
    public double getCameraHeightIn() {
        return 0;
    }

    @Override
    public double getCameraAngleDeg() {
        return 0;
    }

    @Override
    public double getCameraToBumperIn() {
        return 0;
    }

    @Override
    public double ty() {
        return 0;
    }
//    // These are for V2
//    public static double llFOVVertical = 49.7;
//    public static double llFOVHorizontal = 59.6;
//
//    public static final IFieldComponent NONE = new IFieldComponent() {
//        public int id() {return -1;}
//        public double height() {return 0;}
//        public int pipeline() {return 0;}
//        public String toString() { return "NONE"; }
//    };
//
//    public static double kHeightIn = 41.0; // This is approximately correct for all of the hood angles that matter
//    private IFieldComponent mGoal = NONE; // The controller updates this
//    private List<Translation2d> mLowerCorners = new ArrayList<>();
//    private List<Translation2d> mUpperCorners = new ArrayList<>();
//    private final CameraConfig mLimelight;
//    private final Ilite3DSolver mUpperCornerSolver;
//    private final Ilite3DSolver mLowerCornerSolver;
//
//    private final NetworkTable mTable;
//
//    public Limelight(String pNetworkTableName) {
////        mLimelight = LIMELIGHT_V2_LOW_RES.setAddress(pNetworkTableName).setLensHeight(kHeightIn);
////        mUpperCornerSolver = new Ilite3DSolver(
////                mLimelight,
////                Distance.fromInches(OUTER_GOAL_UPPER_CORNERS.height()),
////                Distance.fromInches(OUTER_GOAL_UPPER_CORNERS.width()),
////                Field2022.Distances.TARGETTING_OFFSET.mDistance
////        );
////        mLowerCornerSolver = new Ilite3DSolver(
////                mLimelight,
////                Distance.fromInches(OUTER_GOAL_LOWER_CORNERS.height()),
////                Distance.fromInches(OUTER_GOAL_LOWER_CORNERS.width()),
////                Field2022.Distances.TARGETTING_OFFSET.mDistance
////        );
////        mTable = NetworkTableInstance.getDefault().getTable(pNetworkTableName);
//    }
//
//    @Override
//    public void modeInit(EMatchMode pMode) {
//        db.goaltracking.set(TARGET_ID, NONE.id());
//    }
//
//    @Override
//    public void readInputs() {
//        boolean targetValid = mTable.getEntry("tv").getDouble(Double.NaN) > 0.0;
//        db.goaltracking.set(TV, targetValid);
//        if(mGoal.id() >= 0 && targetValid) {
//            db.goaltracking.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
//            db.goaltracking.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
//            db.goaltracking.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
//            db.goaltracking.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
//            if(db.goaltracking.get(TARGET_ID) != NONE.id()) {
//                // Old way of doing it
//                db.goaltracking.set(TARGET_RANGE_in, calcTargetDistance(mGoal));
//
//                // New way of doing it
//                updateCorners();
//                mUpperCornerSolver.updatePoseToGoal(mUpperCorners);
//                mLowerCornerSolver.updatePoseToGoal(mLowerCorners);
//                if(!mUpperCorners.isEmpty()) {
//                    db.goaltracking.set(T3D_TOP_X_in, mUpperCornerSolver.x().inches());
//                    db.goaltracking.set(T3D_TOP_Y_in, mUpperCornerSolver.y().inches());
//                    db.goaltracking.set(T3D_TOP_AZIMUTH_deg, mUpperCornerSolver.azimuth().degrees());
//                    db.goaltracking.set(T3D_TOP_AZ_OFFSET_deg, mUpperCornerSolver.offsetAzimuth().degrees());
//                    db.goaltracking.set(T3D_TOP_GOAL_RANGE_in, mUpperCornerSolver.range().inches());
//                    db.goaltracking.set(T3D_TOP_LEFT_RANGE_in, mUpperCornerSolver.leftRange().inches());
//                    db.goaltracking.set(T3D_TOP_RIGHT_RANGE_in, mUpperCornerSolver.rightRange().inches());
//                }
//                if(!mLowerCorners.isEmpty()) {
//                    db.goaltracking.set(T3D_BOT_X_in, mLowerCornerSolver.x().inches());
//                    db.goaltracking.set(T3D_BOT_Y_in, mLowerCornerSolver.y().inches());
//                    db.goaltracking.set(T3D_BOT_AZIMUTH_deg, mLowerCornerSolver.azimuth().degrees());
//                    db.goaltracking.set(T3D_BOT_AZ_OFFSET_deg, mLowerCornerSolver.offsetAzimuth().degrees());
//                    db.goaltracking.set(T3D_BOT_GOAL_RANGE_in, mLowerCornerSolver.range().inches());
//                    db.goaltracking.set(T3D_BOT_LEFT_RANGE_in, mLowerCornerSolver.leftRange().inches());
//                    db.goaltracking.set(T3D_BOT_RIGHT_RANGE_in, mLowerCornerSolver.rightRange().inches());
//                }
//            }
//        }
//    }
//
//    @Override
//    public void setOutputs() {
//        setNetworkTableValue("ledMode", LED_MODE);
//        setNetworkTableValue("camMode", CAM_MODE);
//        setNetworkTableValue("snapshot", SNAPSHOT_MODE);
//        setNetworkTableValue("stream", STREAM_MODE);
//
//        mGoal = (db.goaltracking.isSet(TARGET_ID)) ? NONE : db.goaltracking.get(TARGET_ID, Field2022.FieldElement.class);
//        db.goaltracking.set(PIPELINE, mGoal.pipeline());
//        mTable.getEntry("pipeline").setNumber(mGoal.pipeline());
//        // TODO - calibrate this angle. The LL angle may have an offset from the hood, by a few degrees
//        mLimelight.setElevationAngle(db.flywheel.safeGet(EShooterSystemData.HOOD_ANGLE_deg, 0d));
//
//        SmartDashboard.putBoolean("Valid Goal", db.goaltracking.isSet(TV));
//        if(db.goaltracking.isSet(TV)) {
//            SmartDashboard.putNumber("LL Latency (ms)", db.goaltracking.get(TL));
//            SmartDashboard.putNumber("TY Range (in)", db.goaltracking.get(TARGET_RANGE_in));
//            SmartDashboard.putNumber("Upper Corner Range (in)", db.goaltracking.get(T3D_TOP_GOAL_RANGE_in));
//            SmartDashboard.putNumber("Lower Corner Range (in)", db.goaltracking.get(T3D_BOT_GOAL_RANGE_in));
//            SmartDashboard.putNumber("Absolute Azimuth (deg)", db.goaltracking.get(T3D_TOP_AZIMUTH_deg));
//            SmartDashboard.putNumber("Localized X (in)", db.goaltracking.get(T3D_TOP_X_in));
//            SmartDashboard.putNumber("Localized Y (in)", db.goaltracking.get(T3D_TOP_Y_in));
//        }
//    }
//
//    /**
//     * Utility method
//     */
//    private void setNetworkTableValue(String pEntry, EVisionGoal2020 pEnum) {
//        if(db.goaltracking.isSet(pEnum)) {
//            mTable.getEntry(pEntry).setNumber(db.goaltracking.get(pEnum));
//        }
//    }
//
//    @Override
//    public void shutdown() {
//        db.goaltracking.set(EVisionGoal2020.PIPELINE, Limelight.NONE.id());
//        // Force LED off
//        mTable.getEntry("ledMode").setNumber(LimelightLedMode.LED_OFF.ordinal());
//    }
//
//    public String toString() {
//        return db.goaltracking.toCSV();
//    }
//
//    @Override
//    public RobotCodex<EVisionGoal2020> getTargetingData() {
//        return db.goaltracking;
//    }
//
//    @Override
//    public double getCameraHeightIn() {
//        return kHeightIn;
//    }
//
//    @Override
//    public double getCameraAngleDeg() {
//        return mLimelight.elevation_deg();
//    }
//
//    @Override
//    public double getCameraToBumperIn() {
//        return 0d;
//    }
//
//    @Override
//    public double ty() {
//        return 0;
//    }
//
//
//    /**
//     * Credit to FRC254, from their 2019 public code
//     *
//     * Returns raw top-left and top-right corners
//     *
//     * @return list of corners: index 0 - top left, index 1 - top right
//     */
//    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
//    private void updateCorners() {
//        double[] xCorners = mTable.getEntry("tcornx").getDoubleArray(mZeroArray);
//        double[] yCorners = mTable.getEntry("tcorny").getDoubleArray(mZeroArray);
//        boolean tv = mTable.getEntry("tv").getDouble(0) == 1.0;
//
//        // something went wrong
//        if (!tv ||
//                Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
//                || xCorners.length != 8 || yCorners.length != 8) {
//        }
//
//        List<Translation2d> corners = new ArrayList<>();
//        for (int i = 0; i < xCorners.length; i++) {
//            corners.add(new Translation2d(xCorners[i], yCorners[i]));
//        }
//
//        mLowerCorners.clear();
//        mLowerCorners.addAll(getBottomCorners(corners));
//        mUpperCorners.clear();
//        mUpperCorners.addAll(getTopCorners(corners));
//    }
//
//    /**
//     * Credit to FRC254, from their 2019 public code
//     * Returns raw top-left and top-right corners
//     *
//     * @return list of corners: index 0 - top left, index 1 - top right
//     */
//    public List<Translation2d> getTopCorners(List<Translation2d> pBoundingBoxPoints) {
//
//        pBoundingBoxPoints.sort(xSort);
//
//        List<Translation2d> left = pBoundingBoxPoints.subList(0, 4);
//        List<Translation2d> right = pBoundingBoxPoints.subList(4, 8);
//
//        left.sort(ySort);
//        right.sort(ySort);
//
//        List<Translation2d> leftTop = left.subList(0, 2);
//        List<Translation2d> rightTop = right.subList(0, 2);
//
//        leftTop.sort(xSort);
//        rightTop.sort(xSort);
//
//        Translation2d leftCorner = leftTop.get(0);
//        Translation2d rightCorner = rightTop.get(1);
//
//        return List.of(leftCorner, rightCorner);
//    }
//
//    /**
//     * Credit to FRC254, from their 2019 public code
//     * Returns raw top-left and top-right corners
//     *
//     * @return list of corners: index 0 - top left, index 1 - top right
//     */
//    // TODO - this is untested
//    public List<Translation2d> getBottomCorners(List<Translation2d> pBoundingBoxPoints) {
//        pBoundingBoxPoints.sort(xSort);
//
//        List<Translation2d> left = pBoundingBoxPoints.subList(0, 4);
//        List<Translation2d> right = pBoundingBoxPoints.subList(4, 8);
//
//        left.sort(ySort);
//        right.sort(ySort);
//
//        left = left.subList(2,4);
//        right = right.subList(2, 4);
//
//        left.sort(xSort);
//        right.sort(xSort);
//
//        Translation2d leftCorner = left.get(0);
//        Translation2d rightCorner = right.get(1);
//
//        return List.of(leftCorner, rightCorner);
//    }
}