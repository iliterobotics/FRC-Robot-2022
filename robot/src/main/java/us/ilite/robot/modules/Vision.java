package us.ilite.robot.modules;

import com.flybotix.hfr.util.lang.EnumUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.Data;
//import static us.ilite.common.types.ERawLimelightData.*;

import us.ilite.common.Field2020;
import us.ilite.common.IFieldComponent;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ERawLimelightData;
import us.ilite.robot.Robot;

import java.util.Optional;

import static java.lang.Math.*;

public class Vision extends Module {
    private Limelight mLimelight;
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    private Data mData;
    private IFieldComponent mTrackingType;

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
//        mTrackingType = mLimelight.getTracking();
    }
    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {
    }

    @Override
    public void shutdown(double pNow) {

    }

    public void sortTrackingData() {
        Robot.DATA.selectedTarget.reset();

        if (mTrackingType.equals(Field2020.FieldElement.POWER_CELL)) {
            mData.selectedTarget.set(ELimelightData.TV, mData.rawLimelight.get(ERawLimelightData.TV));
            boolean targetValid = Robot.DATA.selectedTarget.isSet(ELimelightData.TV);
            if (targetValid) {
                if (!isTracking) {
                    if (abs(Robot.DATA.rawLimelight.get(ERawLimelightData.TY_0) - Robot.DATA.rawLimelight.get(ERawLimelightData.TY_1)) < acceptableError) {
                        selectedTarget = Robot.DATA.rawLimelight.get(ERawLimelightData.TX_0) > Robot.DATA.rawLimelight.get(ERawLimelightData.TX_1) ? 0 : 1;
                        isTracking = true;
                    } else {
                        selectedTarget = 0;
                    }
                }

                System.out.println("Selected Target" + selectedTarget);
                Robot.DATA.selectedTarget.set(ELimelightData.TX, mTable.getEntry("tx" + selectedTarget).getDouble(Double.NaN) * (Limelight.llFOVHorizontal / 2));
                Robot.DATA.selectedTarget.set(ELimelightData.TY, mTable.getEntry("ty" + selectedTarget).getDouble(Double.NaN) * (Limelight.llFOVVertical / 2));
                Robot.DATA.selectedTarget.set(ELimelightData.TA, mTable.getEntry("ta" + selectedTarget).getDouble(Double.NaN));
                Robot.DATA.selectedTarget.set(ELimelightData.TS, mTable.getEntry("ts" + selectedTarget).getDouble(Double.NaN));
                Robot.DATA.selectedTarget.set(ELimelightData.TL, Robot.DATA.rawLimelight.get(ERawLimelightData.TL));
                Robot.DATA.selectedTarget.set(ELimelightData.TSHORT, mTable.getEntry("tshort" + selectedTarget).getDouble(Double.NaN));
                Robot.DATA.selectedTarget.set(ELimelightData.TLONG, mTable.getEntry("tlong" + selectedTarget).getDouble(Double.NaN));
                Robot.DATA.selectedTarget.set(ELimelightData.THORIZ, mTable.getEntry("thor" + selectedTarget).getDouble(Double.NaN));
                Robot.DATA.selectedTarget.set(ELimelightData.TVERT, mTable.getEntry("tvert" + selectedTarget).getDouble(Double.NaN));
                System.out.println("last tx and ty: " + lastXPosition + ", " + lastYPosition);
                System.out.println("current tx and ty: " + Robot.DATA.selectedTarget.get(ELimelightData.TX) + ", " + Robot.DATA.selectedTarget.get(ELimelightData.TY));
                if (abs(lastXPosition - Robot.DATA.selectedTarget.get(ELimelightData.TX)) > acceptableXError || abs(lastYPosition - Robot.DATA.selectedTarget.get(ELimelightData.TY)) > acceptableYError) {
                    isTracking = false;
                }
                lastXPosition = Robot.DATA.selectedTarget.get(ELimelightData.TX);
                lastYPosition = Robot.DATA.selectedTarget.get(ELimelightData.TY);
            }
        } else {          //set selectedTarget codex straight from limelight codex
            for (ELimelightData e : EnumUtils.getEnums(ELimelightData.class)) {
                Robot.DATA.selectedTarget.set(e, Robot.DATA.limelight.get(e));
            }
//            System.out.println("SELECTED TARGET FROM LIMELIGHT");
        }
    }
}
