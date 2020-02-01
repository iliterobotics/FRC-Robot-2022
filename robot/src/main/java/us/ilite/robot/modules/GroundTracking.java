package us.ilite.robot.modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.IFieldComponent;
import us.ilite.common.types.ELimelightData;
import static us.ilite.common.types.ERawLimelightData.*;
import us.ilite.robot.Robot;

public class GroundTracking extends Module {
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    private IFieldComponent mTrackingType;
    private Limelight mLimelight;
    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {
        Robot.DATA.rawLimelight.reset();
        boolean targetValid = mTable.getEntry("tv").getDouble((Double.NaN)) > 0.0;
        Robot.DATA.rawLimelight.set(tv, targetValid ? 1.0 : null);
//        Robot.DATA.limelight.get(ETrackingType.values()[Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE).intValue()].ordinal());
        if (targetValid) {
            Robot.DATA.rawLimelight.set(tx, mTable.getEntry("tx").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TX_0, mTable.getEntry("tx0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TX_1, mTable.getEntry("tx1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TX_2, mTable.getEntry("tx2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(ty, mTable.getEntry("ty").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TY_0, mTable.getEntry("ty0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TY_1, mTable.getEntry("ty1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TY_2, mTable.getEntry("ty2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(ta, mTable.getEntry("ta").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TA_0, mTable.getEntry("ta0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TA_1, mTable.getEntry("ta1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TA_2, mTable.getEntry("ta2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(ts, mTable.getEntry("ts").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TS_0, mTable.getEntry("ts0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TS_1, mTable.getEntry("ts1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TS_2, mTable.getEntry("ts2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(tl, mTable.getEntry("tl").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(tshort, mTable.getEntry("tshort").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TSHORT_0, mTable.getEntry("tshort0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TSHORT_1, mTable.getEntry("tshort1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TSHORT_2, mTable.getEntry("tshort2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(tlong, mTable.getEntry("tlong").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TLONG_0, mTable.getEntry("tlong0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TLONG_1, mTable.getEntry("tlong1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TLONG_2, mTable.getEntry("tlong2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(thoriz, mTable.getEntry("thor").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(THORIZ_0, mTable.getEntry("thor0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(THORIZ_1, mTable.getEntry("thor1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(THORIZ_2, mTable.getEntry("thor2").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(tvert, mTable.getEntry("tvert").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TVERT_0, mTable.getEntry("tvert0").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TVERT_1, mTable.getEntry("tvert1").getDouble(Double.NaN));
            Robot.DATA.rawLimelight.set(TVERT_2, mTable.getEntry("tvert2").getDouble(Double.NaN));

            Robot.DATA.rawLimelight.set(calcDistToTarget, Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));
            Robot.DATA.rawLimelight.set(calcAngleToTarget, Robot.DATA.limelight.get(ELimelightData.CALC_ANGLE_TO_TARGET));

            if (Robot.DATA.limelight.get(ELimelightData.TARGET_ID) != null) {
             //   Commented out for now until issue with enums isnt fixed
            //    Optional<Translation2d> p = Robot.DATA.limelight.get((Robot.DATA.limelight.get(ELimelightData.TRACKING_TYPE));
          //      if (p.isPresent()) {
                    // Robot.DATA.rawLimelight.set(calcTargetX, p.get().x());
                    // Robot.DATA.rawLimelight.set(calcTargetY, p.get().y());
                }
            }
        }

    }

