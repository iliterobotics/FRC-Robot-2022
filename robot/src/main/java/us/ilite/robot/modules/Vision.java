package us.ilite.robot.modules;

import com.flybotix.hfr.util.lang.EnumUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ERawTargetingData;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ETrackingType;

import java.util.Optional;

import static java.lang.Math.*;

public class Vision extends Module {
    private Limelight mLimelight;
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final Data mData;
    private ETrackingType mTrackingType;

    private double lastXPosition = 0.0;
    private double lastYPosition = 0.0;
    private final double acceptableError = 0.1;
    private final double acceptableXError = 10.0;
    private final double acceptableYError = 10.0;
    private int selectedTarget = -1;

    private boolean isTracking = false;

    public Vision(Data pData, Limelight pLimelight) {
        mData = pData;
        mLimelight = pLimelight;
        mTrackingType = mLimelight.getTracking();
    }
    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {
        mData.rawLimelight.reset();
        boolean targetValid = mTable.getEntry("tv").getDouble((Double.NaN)) > 0.0;
        mData.rawLimelight.set(ERawTargetingData.tv, targetValid ? 1.0 : null);
        mTrackingType = mLimelight.getTracking();
        if (targetValid) {
            mData.rawLimelight.set(ERawTargetingData.tx, mTable.getEntry("tx").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tx0, mTable.getEntry("tx0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tx1, mTable.getEntry("tx1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tx2, mTable.getEntry("tx2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ty, mTable.getEntry("ty").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ty0, mTable.getEntry("ty0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ty1, mTable.getEntry("ty1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ty2, mTable.getEntry("ty2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ta, mTable.getEntry("ta").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ta0, mTable.getEntry("ta0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ta1, mTable.getEntry("ta1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ta2, mTable.getEntry("ta2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ts, mTable.getEntry("ts").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ts0, mTable.getEntry("ts0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ts1, mTable.getEntry("ts1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.ts2, mTable.getEntry("ts2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tl, mTable.getEntry("tl").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tshort, mTable.getEntry("tshort").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tshort0, mTable.getEntry("tshort0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tshort1, mTable.getEntry("tshort1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tshort2, mTable.getEntry("tshort2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tlong, mTable.getEntry("tlong").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tlong0, mTable.getEntry("tlong0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tlong1, mTable.getEntry("tlong1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tlong2, mTable.getEntry("tlong2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.thoriz, mTable.getEntry("thor").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.thoriz0, mTable.getEntry("thor0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.thoriz1, mTable.getEntry("thor1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.thoriz2, mTable.getEntry("thor2").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tvert, mTable.getEntry("tvert").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tvert0, mTable.getEntry("tvert0").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tvert1, mTable.getEntry("tvert1").getDouble(Double.NaN));
            mData.rawLimelight.set(ERawTargetingData.tvert2, mTable.getEntry("tvert2").getDouble(Double.NaN));

            mData.rawLimelight.set(ERawTargetingData.targetOrdinal, mData.limelight.get(ELimelightData.targetOrdinal));
            mData.rawLimelight.set(ERawTargetingData.calcDistToTarget, mData.limelight.get(ELimelightData.calcDistToTarget));
            mData.rawLimelight.set(ERawTargetingData.calcAngleToTarget, mData.limelight.get(ELimelightData.calcAngleToTarget));

            if (mLimelight.mVisionTarget != null) {
                Optional<Translation2d> p = mLimelight.calcTargetLocation(mLimelight.mVisionTarget);
                if (p.isPresent()) {
                   // mData.rawLimelight.set(ERawTargetingData.calcTargetX, p.get().x());
                   // mData.rawLimelight.set(ERawTargetingData.calcTargetY, p.get().y());
                }
            }
        }

    }

    @Override
    public void shutdown(double pNow) {

    }

    public void sortTrackingData() {
        mData.selectedTarget.reset();

        if (mTrackingType.equals(ETrackingType.BALL)) {
            mData.selectedTarget.set(ELimelightData.tv, mData.rawLimelight.get(ERawTargetingData.tv));
            boolean targetValid = mData.selectedTarget.isSet(ELimelightData.tv);
            if (targetValid) {
                if (!isTracking) {
                    if (abs(mData.rawLimelight.get(ERawTargetingData.ty0) - mData.rawLimelight.get(ERawTargetingData.ty1)) < acceptableError) {
                        selectedTarget = mData.rawLimelight.get(ERawTargetingData.tx0) > mData.rawLimelight.get(ERawTargetingData.tx1) ? 0 : 1;
                        isTracking = true;
                    } else {
                        selectedTarget = 0;
                    }
                }

                System.out.println("Selected Target" + selectedTarget);
                mData.selectedTarget.set(ELimelightData.tx, mTable.getEntry("tx" + selectedTarget).getDouble(Double.NaN) * (Limelight.llFOVHorizontal / 2));
                mData.selectedTarget.set(ELimelightData.ty, mTable.getEntry("ty" + selectedTarget).getDouble(Double.NaN) * (Limelight.llFOVVertical / 2));
                mData.selectedTarget.set(ELimelightData.ta, mTable.getEntry("ta" + selectedTarget).getDouble(Double.NaN));
                mData.selectedTarget.set(ELimelightData.ts, mTable.getEntry("ts" + selectedTarget).getDouble(Double.NaN));
                mData.selectedTarget.set(ELimelightData.tl, mData.rawLimelight.get(ERawTargetingData.tl));
                mData.selectedTarget.set(ELimelightData.tshort, mTable.getEntry("tshort" + selectedTarget).getDouble(Double.NaN));
                mData.selectedTarget.set(ELimelightData.tlong, mTable.getEntry("tlong" + selectedTarget).getDouble(Double.NaN));
                mData.selectedTarget.set(ELimelightData.thoriz, mTable.getEntry("thor" + selectedTarget).getDouble(Double.NaN));
                mData.selectedTarget.set(ELimelightData.tvert, mTable.getEntry("tvert" + selectedTarget).getDouble(Double.NaN));

                mData.selectedTarget.set(ELimelightData.targetOrdinal, mData.rawLimelight.get(ERawTargetingData.targetOrdinal));
                mData.selectedTarget.set(ELimelightData.calcDistToTarget, mData.rawLimelight.get(ERawTargetingData.calcDistToTarget));
                mData.selectedTarget.set(ELimelightData.calcAngleToTarget, mData.rawLimelight.get(ERawTargetingData.calcAngleToTarget));

                mData.selectedTarget.set(ELimelightData.calcTargetX, mData.rawLimelight.get(ERawTargetingData.calcTargetX));
                mData.selectedTarget.set(ELimelightData.calcTargetY, mData.rawLimelight.get(ERawTargetingData.calcTargetY));

                System.out.println("last tx and ty: " + lastXPosition + ", " + lastYPosition);
                System.out.println("current tx and ty: " + mData.selectedTarget.get(ELimelightData.tx) + ", " + mData.selectedTarget.get(ELimelightData.ty));
                if (abs(lastXPosition - mData.selectedTarget.get(ELimelightData.tx)) > acceptableXError || abs(lastYPosition - mData.selectedTarget.get(ELimelightData.ty)) > acceptableYError) {
                    isTracking = false;
                }
                lastXPosition = mData.selectedTarget.get(ELimelightData.tx);
                lastYPosition = mData.selectedTarget.get(ELimelightData.ty);
            }
        } else {          //set selectedTarget codex straight from limelight codex
            for (ELimelightData e : EnumUtils.getEnums(ELimelightData.class)) {
                mData.selectedTarget.set(e, mData.limelight.get(e));
            }
//            System.out.println("SELECTED TARGET FROM LIMELIGHT");
        }
    }
}
