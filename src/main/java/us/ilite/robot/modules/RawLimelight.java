package us.ilite.robot.modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Field2022;
import us.ilite.common.IFieldComponent;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.ERawLimelightData.*;

/**
 * A module for interfacing with the Ground Limelight
 */
public class RawLimelight extends Module{
    public static final IFieldComponent NONE = new IFieldComponent() {
        public int id() {return -1;}
        public double height() {return 0;}
        public int pipeline() {return 0;}
        public String toString() { return "NONE"; }
    };

    // =============================================================================
    // LimeLight Camera Constants
    // Note: These constants need to be recalculated for a specific robot geometry
    // =============================================================================
    public static double kHeightIn = 0.0;   //TODO These constants need to be remeasured
    public static double kToBumperIn = 0.0;
    public static double kAngleDeg = 0.0;

    public static double llFOVVertical = 49.7;
    public static double llFOVHorizontal = 59.6;

    // Left angle coefficients for angle = a + bx + cx^2
    //    a	0.856905324060421
    //    b	-3.01414088331715
    //    c	-0.0331854848038372
    public static double kLeftACoeff = 0.856905324060421;
    public static double kLeftBCoeff = -3.01414088331715;
    public static double kLeftCCoeff = -0.0331854848038372;

    // Right angle coefficients for angle = a + bx + cx^2
    // a	-54.3943883842204
    // b	-4.53956454545558
    // c	-0.0437470770400814
    public static double kRightACoeff = -54.3943883842204;
    public static double kRightBCoeff = -4.53956454545558;
    public static double kRightCCoeff = -0.0437470770400814;

    private final NetworkTable mTable;

    protected IFieldComponent mVisionTarget;

    public RawLimelight(String pNetworkTableName) {
        mTable = NetworkTableInstance.getDefault().getTable(pNetworkTableName);
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        db.rawLimelight.set(TARGET_ID, (double) NONE.id());
    }

    @Override
    protected void readInputs() {
        mVisionTarget = Field2022.FieldElement.values()[(int) db.rawLimelight.get(TARGET_ID)];

        boolean targetValid = mTable.getEntry("tv").getDouble(0.0) > 0.0;
        db.rawLimelight.set(TV, targetValid ? 1.0d : null);

        if(targetValid) {
            db.rawLimelight.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
            db.rawLimelight.set(TX_0, mTable.getEntry("tx0").getDouble(Double.NaN));
            db.rawLimelight.set(TX_1, mTable.getEntry("tx1").getDouble(Double.NaN));
            db.rawLimelight.set(TX_2, mTable.getEntry("tx2").getDouble(Double.NaN));
            db.rawLimelight.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
            db.rawLimelight.set(TY_0,mTable.getEntry("ty0").getDouble(Double.NaN));
            db.rawLimelight.set(TY_1,mTable.getEntry("ty1").getDouble(Double.NaN));
            db.rawLimelight.set(TY_2,mTable.getEntry("ty2").getDouble(Double.NaN));
            db.rawLimelight.set(TA,mTable.getEntry("ta").getDouble(Double.NaN));
            db.rawLimelight.set(TA_0,mTable.getEntry("ta0").getDouble(Double.NaN));
            db.rawLimelight.set(TA_1,mTable.getEntry("ta1").getDouble(Double.NaN));
            db.rawLimelight.set(TA_2,mTable.getEntry("ta2").getDouble(Double.NaN));
            db.rawLimelight.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
            db.rawLimelight.set(TS_0,mTable.getEntry("ts0").getDouble(Double.NaN));
            db.rawLimelight.set(TS_1,mTable.getEntry("ts1").getDouble(Double.NaN));
            db.rawLimelight.set(TS_2,mTable.getEntry("ts2").getDouble(Double.NaN));
            db.rawLimelight.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
            db.rawLimelight.set(TSHORT,mTable.getEntry("tshort").getDouble(Double.NaN));
            db.rawLimelight.set(TSHORT_0,mTable.getEntry("tshort0").getDouble(Double.NaN));
            db.rawLimelight.set(TSHORT_1,mTable.getEntry("tshort1").getDouble(Double.NaN));
            db.rawLimelight.set(TSHORT_2,mTable.getEntry("tshort2").getDouble(Double.NaN));
            db.rawLimelight.set(TLONG,mTable.getEntry("tlong").getDouble(Double.NaN));
            db.rawLimelight.set(TLONG_0,mTable.getEntry("tlong0").getDouble(Double.NaN));
            db.rawLimelight.set(TLONG_1,mTable.getEntry("tlong1").getDouble(Double.NaN));
            db.rawLimelight.set(TLONG_2,mTable.getEntry("tlong2").getDouble(Double.NaN));
            db.rawLimelight.set(THORIZ,mTable.getEntry("thoriz").getDouble(Double.NaN));
            db.rawLimelight.set(THORIZ_0,mTable.getEntry("thoriz0").getDouble(Double.NaN));
            db.rawLimelight.set(THORIZ_1,mTable.getEntry("thoriz1").getDouble(Double.NaN));
            db.rawLimelight.set(THORIZ_2,mTable.getEntry("thoriz2").getDouble(Double.NaN));
            db.rawLimelight.set(TVERT,mTable.getEntry("tvert").getDouble(Double.NaN));
            db.rawLimelight.set(TVERT_0,mTable.getEntry("tvert0").getDouble(Double.NaN));
            db.rawLimelight.set(TVERT_1,mTable.getEntry("tvert1").getDouble(Double.NaN));
            db.rawLimelight.set(TVERT_2,mTable.getEntry("tvert2").getDouble(Double.NaN));
//            if(mVisionTarget.equals(NONE)) {
//                db.rawLimelight.set(CALC_DIST_TO_TARGET, calcTargetDistance(mVisionTarget.height()));
//                db.rawLimelight.set(CALC_ANGLE_TO_TARGET, calcTargetApproachAngle());
//                Optional<Translation2d> p = calcTargetLocation(mVisionTarget);
//                if(p.isPresent()) {
//                    db.rawLimelight.set(CALC_TARGET_X, p.get().getX());
//                    db.rawLimelight.set(CALC_TARGET_Y, p.get().getY());
//                }
//            }
        }

    }

    @Override
    protected void setOutputs() {
        setLedMode();
        setCamMode();
        setStreamMode();
        setSnapshotMode();
        setPipeline();
    }

    private void setPipeline() {
        mTable.getEntry("pipeline").setNumber(db.rawLimelight.get(mVisionTarget.pipeline()));
    }

    private void setLedMode() {
        mTable.getEntry("ledMode").setNumber(db.rawLimelight.get(LED_MODE));
    }

    private void setCamMode() {
        mTable.getEntry("camMode").setNumber(db.rawLimelight.get(CAM_MODE));
    }

    private void setStreamMode() {
        mTable.getEntry("stream").setNumber(db.rawLimelight.get(STREAM_MODE));
    }

    private void setSnapshotMode() {
        mTable.getEntry("snapshot").setNumber(db.rawLimelight.get(SNAPSHOT_MODE));
    }

    public String toString() {
        return db.limelight.toCSV();
    }

}