package us.ilite.robot.modules;

import java.util.Optional;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.Field2020;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.IFieldComponent;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.targetData.ITargetDataProvider;
import static us.ilite.common.types.ELimelightData.*;

/**
 * A module for interfacing with the Goal Limelight
 */
public class Limelight extends Module implements ITargetDataProvider {
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
    public static double kHeightIn = 31.75;   //Measurements from Bunnybot for skew testing
    public static double kToBumperIn = 22.125;
    public static double kAngleDeg = 24.85;

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

    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable(Settings.kFlywheelLimelightNetworkTable);


    public Limelight() {
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        Robot.DATA.goaltracking.set(TARGET_ID, NONE.id());
    }

    @Override
    public void readInputs(double pNow) {
        boolean targetValid = mTable.getEntry("tv").getDouble(Double.NaN) > 0.0;
        Robot.DATA.goaltracking.set(TV, targetValid ? 1.0d : 0d);
        if(targetValid) {
            Robot.DATA.goaltracking.set(TX, mTable.getEntry("tx").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TY,mTable.getEntry("ty").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TA,mTable.getEntry("ta").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TS,mTable.getEntry("ts").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TL,mTable.getEntry("tl").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TSHORT,mTable.getEntry("tshort").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TLONG,mTable.getEntry("tlong").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(THORIZ,mTable.getEntry("thoriz").getDouble(Double.NaN));
            Robot.DATA.goaltracking.set(TVERT,mTable.getEntry("tvert").getDouble(Double.NaN));
            if(Robot.DATA.goaltracking.get(TARGET_ID) != -1) {
                Robot.DATA.goaltracking.set(CALC_DIST_TO_TARGET, calcTargetDistance(Field2020.FieldElement.values()[(int) Robot.DATA.goaltracking.get(TARGET_ID)].height()));
                Robot.DATA.goaltracking.set(CALC_ANGLE_TO_TARGET, calcTargetApproachAngle());
                Optional<Translation2d> p = calcTargetLocation(Field2020.FieldElement.values()[(int) Robot.DATA.goaltracking.get(TARGET_ID)]);
                if(p.isPresent()) {
                    Robot.DATA.goaltracking.set(CALC_TARGET_X, p.get().getX());
                    Robot.DATA.goaltracking.set(CALC_TARGET_Y, p.get().getY());
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
        setPipeline();
        Robot.DATA.goaltracking.set(ANGLE_FROM_HORIZON, kAngleDeg);
//        Robot.DATA.limelight.set(ANGLE_FROM_HORIZON, Robot.DATA.flywheel.get(ANGLE_FROM_HORIZON)); //TODO Add angle functionality to flywheel module
    }

    @Override
    public void shutdown(double pNow) {
        db.goaltracking.set(ELimelightData.PIPELINE, Limelight.NONE.id());
    }

    private void setPipeline() {
        int mPipeline = (Robot.DATA.goaltracking.get(TARGET_ID) == -1) ?
                        NONE.pipeline() :
                        Field2020.FieldElement.values()[(int) Robot.DATA.goaltracking.get(TARGET_ID)].pipeline();

        Robot.DATA.goaltracking.set(PIPELINE, mPipeline);
        mTable.getEntry("pipeline").setNumber(Robot.DATA.goaltracking.get(PIPELINE));
    }

    private void setLedMode() {
        mTable.getEntry("ledMode").setNumber(Robot.DATA.goaltracking.get(LED_MODE));
    }

    private void setCamMode() {
        mTable.getEntry("camMode").setNumber(Robot.DATA.goaltracking.get(CAM_MODE));
    }

    private void setStreamMode() {
        mTable.getEntry("stream").setNumber(Robot.DATA.goaltracking.get(STREAM_MODE));
    }

    private void setSnapshotMode() {
        mTable.getEntry("snapshot").setNumber(Robot.DATA.goaltracking.get(SNAPSHOT_MODE));
    }

    public String toString() {
        return Robot.DATA.goaltracking.toCSV();
    }

    @Override
    public RobotCodex<ELimelightData> getTargetingData() {
        return Robot.DATA.goaltracking;
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