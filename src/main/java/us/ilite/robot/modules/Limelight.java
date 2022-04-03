package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.IFieldComponent;

//import static us.ilite.common.Field2022.FieldElement.OUTER_GOAL_LOWER_CORNERS;
//import static us.ilite.common.Field2022.FieldElement.OUTER_GOAL_UPPER_CORNERS;
import static us.ilite.common.types.ELimelightData.T3D_TOP_Y_in;

import us.ilite.robot.Enums;
import us.ilite.robot.modules.targetData.ITargetDataProvider;
import us.ilite.robot.vision.CameraConfig;
import us.ilite.robot.vision.Ilite3DSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ilite.common.types.ELimelightData.*;
import static us.ilite.common.types.ERawLimelightData.TARGET_ID;
import static us.ilite.robot.vision.CameraConfig.*;
import static us.ilite.robot.vision.Ilite3DSolver.xSort;
import static us.ilite.robot.vision.Ilite3DSolver.ySort;

/**
 * A module for interfacing with the Goal Limelight
 */
public class Limelight extends Module implements ITargetDataProvider {
    // These are for V2
    public static double llFOVVertical = 49.7; // TODO Confirm if this is true
    public static double llFOVHorizontal = 59.6;

    public static final IFieldComponent NONE = new IFieldComponent() {
        public int id() {return -1;}
        public double height() {return 0;}
        public int pipeline() {return 0;}
        public String toString() { return "NONE"; }
    };

    public static double kHeightIn = 41.0; // TODO Adjust based on 2022 limelight placement
    private IFieldComponent mGoal = NONE; // Updates based on TARGET_ID

    private final NetworkTable mTable;

    public Limelight() {
        mTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void readInputs() {
        boolean targetValid = mTable.getEntry("tv").getDouble(0d) > 0d;

        db.limelight.set(TV, targetValid);
        db.limelight.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
        db.limelight.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
        db.limelight.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
        db.limelight.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
    }

    @Override
    public void setOutputs() {
        mGoal = Field2022.FieldElement.values()[(int) db.limelight.get(ELimelightData.TARGET_ID)];
        db.limelight.set(PIPELINE, mGoal.pipeline());

        setNetworkTableValue("ledMode", LED_MODE);
        setNetworkTableValue("camMode", CAM_MODE);
        setNetworkTableValue("snapshot", SNAPSHOT_MODE);
        setNetworkTableValue("stream", STREAM_MODE);
        setNetworkTableValue("pipeline", PIPELINE);

        SmartDashboard.putNumber("Logged TX", db.limelight.get(TX));
        SmartDashboard.putBoolean("Logged TV", db.limelight.isSet(TV));
        SmartDashboard.putNumber("Logged Pipeline", db.limelight.get(PIPELINE));
    }

    /**
     * Utility method
     */
    private void setNetworkTableValue(String pEntry, ELimelightData pEnum) {
        mTable.getEntry(pEntry).setNumber(db.limelight.get(pEnum));
    }

    @Override
    public void shutdown() {
        db.limelight.set(ELimelightData.PIPELINE, Limelight.NONE.id());
        // Force LED off
//        mTable.getEntry("ledMode").setNumber(Enums.LimelightLedMode.LED_OFF.ordinal());
    }

    public String toString() {
        return db.limelight.toCSV();
    }

    @Override
    public RobotCodex<ELimelightData> getTargetingData() {
        return null;
    }

    @Override
    public double getCameraHeightIn() {
        return 0;
    }

    @Override
    public double getCameraAngleDeg() {
//        return mLimelight.elevation_deg();
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

    /**
     * Credit to FRC254, from their 2019 public code
     *
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private void updateCorners() {
        double[] xCorners = mTable.getEntry("tcornx").getDoubleArray(mZeroArray);
        double[] yCorners = mTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        boolean tv = mTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!tv ||
                Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
                || xCorners.length != 8 || yCorners.length != 8) {
        }

        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }
    }

    /**
     * Credit to FRC254, from their 2019 public code
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public List<Translation2d> getTopCorners(List<Translation2d> pBoundingBoxPoints) {

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
    public List<Translation2d> getBottomCorners(List<Translation2d> pBoundingBoxPoints) {
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