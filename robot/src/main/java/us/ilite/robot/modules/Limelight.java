package us.ilite.robot.modules;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.Field2020;
import us.ilite.common.IFieldComponent;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

import java.util.Optional;

import static us.ilite.common.types.ELimelightData.*;

public class Limelight extends Module implements ITargetDataProvider {
    public static final IFieldComponent NONE = new IFieldComponent() {
        public int id() {return -1;}
        public double height() {return 0;}
        public int pipeline() {return 0;}
        public String toString() { return "NONE"; }
    };

    private final ILog mLog = Logger.createLog(Limelight.class);
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");


    // =============================================================================
    // LimeLight Camera Constants
    // Note: These constants need to be recalculated for a specific robot geometry
    // =============================================================================
    public static double kHeightIn = 58.0;
    public static double kToBumperIn = 10.0;
    public static double kAngleDeg = 28.55;

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

    protected IFieldComponent mVisionTarget = null;

    public Limelight() {
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        setTracking(NONE);
    }

    @Override
    public void readInputs(double pNow) {
        mVisionTarget = Field2020.FieldElement.values()[Robot.DATA.limelight.get(TARGET_ID).intValue()];

        boolean targetValid = mTable.getEntry("tv").getDouble(0.0) > 0.0;
        Robot.DATA.limelight.set(TV, targetValid ? 1.0d : null);

        if(targetValid) {
            Robot.DATA.limelight.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TA,mTable.getEntry("ta").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TSHORT,mTable.getEntry("tshort").getDouble(Double.NaN));
            Robot.DATA.limelight.set(TLONG,mTable.getEntry("tlong").getDouble(Double.NaN));
            Robot.DATA.limelight.set(THORIZ,mTable.getEntry("thoriz").getDouble(Double.NaN));
            Robot. DATA.limelight.set(TVERT,mTable.getEntry("tvert").getDouble(Double.NaN));
            if(mVisionTarget.equals(NONE)) {
                Robot.DATA.limelight.set(CALC_DIST_TO_TARGET, calcTargetDistance(mVisionTarget.height()));
                Robot.DATA.limelight.set(CALC_ANGLE_TO_TARGET, calcTargetApproachAngle());
                Optional<Translation2d> p = calcTargetLocation(mVisionTarget);
                if(p.isPresent()) {
                    Robot.DATA.limelight.set(CALC_TARGET_X, p.get().getX());
                    Robot.DATA.limelight.set(CALC_TARGET_Y, p.get().getY());
                }
            }
        }
    }

    @Override
    public void setOutputs(double pNow) {
        setLedMode();
        setCamMode();
        setStreamMode();
        setSnapshotMode();
    }

    @Override
    public void shutdown(double pNow) {

    }

    private void setPipeline(int pipeline) {
        mTable.getEntry("pipeline").setNumber(pipeline);
    }

    private void setTracking(IFieldComponent pFieldElement) {
        mVisionTarget = pFieldElement;
    }

    private void setLedMode() {
        if (!Robot.DATA.limelight.get(DESIRED_LED_MODE).equals(Robot.DATA.limelight.get(CURRENT_LED_MODE))) {
            mTable.getEntry("ledMode").setNumber(Robot.DATA.limelight.get(DESIRED_LED_MODE));
        }
    }
    private void setCamMode() {
        if (!Robot.DATA.limelight.get(DESIRED_CAM_MODE).equals(Robot.DATA.limelight.get(CURRENT_CAM_MODE))) {
            mTable.getEntry("camMode").setNumber(Robot.DATA.limelight.get(DESIRED_CAM_MODE));
        }
    }

    private void setStreamMode() {
        if (!Robot.DATA.limelight.get(DESIRED_STREAM_MODE).equals(Robot.DATA.limelight.get(CURRENT_STREAM_MODE))) {
            mTable.getEntry("stream").setNumber(Robot.DATA.limelight.get(DESIRED_STREAM_MODE));
        }
    }

    private void setSnapshotMode() {
        if (!Robot.DATA.limelight.get(DESIRED_SNAPSHOT_MODE).equals(Robot.DATA.limelight.get(CURRENT_SNAPSHOT_MODE))) {
            mTable.getEntry("snapshot").setNumber(Robot.DATA.limelight.get(DESIRED_SNAPSHOT_MODE));
        }
    }

    public String toString() {
        return Robot.DATA.limelight.toCSV();
    }

    @Override
    public Codex<Double, ELimelightData> getTargetingData() {
        return Robot.DATA.limelight;
    }

    @Override
    public double getCameraHeightIn() {
        return kHeightIn;
    }

    @Override
    public double getCameraAngleDeg() {
        return kAngleDeg;
    }

    @Override
    public double getCameraToBumperIn() {
        return kToBumperIn;
    }

    @Override
    public double getLeftCoeffA() {
        return kLeftACoeff;
    }

    @Override
    public double getLeftCoeffB() {
        return kLeftBCoeff;
    }

    @Override
    public double getLeftCoeffC() {
        return kLeftCCoeff;
    }

    @Override
    public double getRightCoeffA() {
        return kRightACoeff;
    }

    @Override
    public double getRightCoeffB() {
        return kRightBCoeff;
    }

    @Override
    public double getRightCoeffC() {
        return kRightCCoeff;
    }

}