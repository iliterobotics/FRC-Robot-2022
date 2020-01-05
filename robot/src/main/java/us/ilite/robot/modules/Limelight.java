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
import us.ilite.common.types.ETargetingData;
import us.ilite.common.types.ETrackingType;

import static us.ilite.common.types.ETargetingData.*;

import us.ilite.robot.loops.Loop;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

public class Limelight extends Loop implements ITargetDataProvider {

    private final ILog mLog = Logger.createLog(Limelight.class);
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");

    private final Data mData;

    private ETrackingType mTrackingType = null;
    private VisionTarget mVisionTarget = null;

    public Limelight(Data pData) {
        this.mData = pData;
    }

    @Override
    public void modeInit(double pNow) {
        setTracking(ETrackingType.NONE);
    }

    @Override
    public void periodicInput(double pNow) {
        mData.limelight.reset();
        boolean targetValid = mTable.getEntry("tv").getDouble(0.0) > 0.0;
        mData.limelight.set(ETargetingData.tv, targetValid ? 1.0d : null);

        if(targetValid) {
            mData.limelight.set(ETargetingData.tx, mTable.getEntry("tx").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.ty,mTable.getEntry("ty").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.ta,mTable.getEntry("ta").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.ts,mTable.getEntry("ts").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.tl,mTable.getEntry("tl").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.tshort,mTable.getEntry("tshort").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.tlong,mTable.getEntry("tlong").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.thoriz,mTable.getEntry("thoriz").getDouble(Double.NaN));
            mData.limelight.set(ETargetingData.tvert,mTable.getEntry("tvert").getDouble(Double.NaN));
            if(mVisionTarget != null) {

                mData.limelight.set(ETargetingData.targetOrdinal, (double)mVisionTarget.ordinal());
                mData.limelight.set(ETargetingData.calcDistToTarget, calcTargetDistance(mVisionTarget));
                mData.limelight.set(calcAngleToTarget, calcTargetApproachAngle());
                Optional<Translation2d> p = calcTargetLocation(mVisionTarget);
                if(p.isPresent()) {
                    mData.limelight.set(ETargetingData.calcTargetX, p.get().getX());
                    mData.limelight.set(ETargetingData.calcTargetY, p.get().getY());
                }
            }
        }
    }

    @Override
    public void update(double pNow) {

        mLog.error("Current Tracking Type: " + (mTrackingType == null ? "Null" : mTrackingType.name()));
        if(mTrackingType != null) {
            setLedMode(mTrackingType.getLedOn() ? LedMode.LED_ON : LedMode.LED_OFF);
            setPipeline(mTrackingType.getPipeline());
        } else {
            setTracking(ETrackingType.NONE);
        }
    }

    public void loop(double pNow) {
        update(pNow);
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
        return mData.limelight.toCSV();
    }

    @Override
    public Codex<Double, ETargetingData> getTargetingData() {
        return mData.limelight;
    }

    @Override
    public double getCameraHeightIn() {
        return Settings.LimeLight.kHeightIn;
    }

    @Override
    public double getCameraAngleDeg() {
        return Settings.LimeLight.kAngleDeg;
    }

    @Override
    public double getCameraToBumperIn() {
        return Settings.LimeLight.kToBumperIn;
    }

    @Override
    public double getLeftCoeffA() {
        return Settings.LimeLight.kLeftACoeff;
    }

    @Override
    public double getLeftCoeffB() {
        return Settings.LimeLight.kLeftBCoeff;
    }

    @Override
    public double getLeftCoeffC() {
        return Settings.LimeLight.kLeftCCoeff;
    }

    @Override
    public double getRightCoeffA() {
        return Settings.LimeLight.kRightACoeff;
    }

    @Override
    public double getRightCoeffB() {
        return Settings.LimeLight.kRightBCoeff;
    }

    @Override
    public double getRightCoeffC() {
        return Settings.LimeLight.kRightCCoeff;
    }

}