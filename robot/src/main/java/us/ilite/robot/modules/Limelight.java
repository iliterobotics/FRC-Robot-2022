package us.ilite.robot.modules;

import java.util.Optional;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.geometry.Translation2d;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.config.Settings.VisionTarget;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;

import static us.ilite.common.types.ETargetingData.*;

import us.ilite.robot.Robot;
import us.ilite.robot.loops.Loop;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

public class Limelight extends Loop implements ITargetDataProvider {

    private final ILog mLog = Logger.createLog(Limelight.class);
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");

    // =============================================================================
    // LimeLight Camera Constants
    // Note: These constants need to be recalculted for a specific robot geometry
    // =============================================================================
    public static double kHeightIn = 58.0;
    public static double kToBumperIn = 10.0;
    public static double kAngleDeg = 28.55;

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

    private ETrackingType mTrackingType = null;
    private VisionTarget mVisionTarget = null;

    public Limelight(Data pData) {

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        setTracking(ETrackingType.NONE);
    }

    @Override
    public void readInputs(double pNow) {
        boolean targetValid = mTable.getEntry("tv").getDouble(0.0) > 0.0;
        Robot.DATA.limelight.set(ETargetingData.tv, targetValid ? 1.0d : null);

        if(targetValid) {
            Robot.DATA.limelight.set(ETargetingData.tx, mTable.getEntry("tx").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.ty,mTable.getEntry("ty").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.ta,mTable.getEntry("ta").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.ts,mTable.getEntry("ts").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.tl,mTable.getEntry("tl").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.tshort,mTable.getEntry("tshort").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.tlong,mTable.getEntry("tlong").getDouble(Double.NaN));
            Robot.DATA.limelight.set(ETargetingData.thoriz,mTable.getEntry("thoriz").getDouble(Double.NaN));
            Robot. DATA.limelight.set(ETargetingData.tvert,mTable.getEntry("tvert").getDouble(Double.NaN));
            if(mVisionTarget != null) {

                Robot.DATA.limelight.set(ETargetingData.targetOrdinal, (double)mVisionTarget.ordinal());
                Robot.DATA.limelight.set(ETargetingData.calcDistToTarget, calcTargetDistance(mVisionTarget));
                Robot.DATA.limelight.set(calcAngleToTarget, calcTargetApproachAngle());
                Optional<Translation2d> p = calcTargetLocation(mVisionTarget);
                if(p.isPresent()) {
                    Robot.DATA.limelight.set(ETargetingData.calcTargetX, p.get().getX());
                    Robot.DATA.limelight.set(ETargetingData.calcTargetY, p.get().getY());
                }
            }
        }
    }

    @Override
    public void setOutputs(double pNow) {

        mLog.error("Current Tracking Type: " + (mTrackingType == null ? "Null" : mTrackingType.name()));
        if(mTrackingType != null) {
            setLedMode(mTrackingType.getLedOn() ? LedMode.LED_ON : LedMode.LED_OFF);
            setPipeline(mTrackingType.getPipeline());
        } else {
            setTracking(ETrackingType.NONE);
        }
    }

    public void loop(double pNow) {
        setOutputs(pNow);
    }

    @Override
    public void shutdown(double pNow) {

    }

    public enum LedMode {
        NO_CHANGE,
        LED_OFF,
        LED_BLINK,
        LED_ON;
    }

    public enum Stream {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY;
    }

    public void setVisionTarget(VisionTarget pVisionTarget) {
        mVisionTarget = pVisionTarget;
        // TODO reconcile pipeline
    }

    public void setTracking(ETrackingType pTrackingType) {
        mLog.error("SET TRACKING TYPE: " + pTrackingType.name());
        mTrackingType = pTrackingType;
        // TODO - reconcile pipeline
    }
    
    public ETrackingType getTracking() {
        return this.mTrackingType;
    }

    public void setCamMode(boolean pMode) {
        mTable.getEntry("camMode").setBoolean(pMode);
    }

    public void setLedMode(LedMode pMode) {
        mTable.getEntry("ledMode").setNumber(pMode.ordinal());
    }

    public void setPipeline(int pipeline) {
        mTable.getEntry("pipeline").setNumber(pipeline);
    }

    public void setSnapshot(boolean snapshot) {
        mTable.getEntry("snapshot").setBoolean(snapshot);
    }

    public void setStream(Stream stream) { 
        mTable.getEntry("stream").setNumber(stream.ordinal());
    }


    @Override
    public String toString() {
        return Robot.DATA.limelight.toCSV();
    }

    @Override
    public Codex<Double, ETargetingData> getTargetingData() {
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