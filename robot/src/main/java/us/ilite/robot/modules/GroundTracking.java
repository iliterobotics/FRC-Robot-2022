package us.ilite.robot.modules;

import com.flybotix.hfr.util.lang.EnumUtils;
import us.ilite.common.types.ELimelightData;

import static java.lang.Math.abs;
import static us.ilite.common.types.ERawLimelightData.*;

import us.ilite.common.types.ERawLimelightData;
import us.ilite.robot.Robot;

import java.util.Arrays;
import java.util.stream.Collectors;

public class GroundTracking extends Module {
    private double mLastXPosition = 0.0;
    private double mLastYPosition = 0.0;
    private final double mAcceptableError = 0.1;
    private final double mAcceptableXError = 10.0;
    private final double mAcceptableYError = 10.0;
    private int mSelectedTarget = -1;
    private boolean mIsTracking = false;

    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {
        sortTrackingData();
    }

    public void sortTrackingData() {
            Robot.DATA.groundTracking.set(ELimelightData.TV, Robot.DATA.rawLimelight.get(ERawLimelightData.TV));
            boolean targetValid = Robot.DATA.groundTracking.isSet(ELimelightData.TV);
            if (targetValid) {
                if (!mIsTracking) {
                    if (abs(Robot.DATA.rawLimelight.get(ERawLimelightData.TY_0) - Robot.DATA.rawLimelight.get(ERawLimelightData.TY_1)) < mAcceptableError) {
                        mSelectedTarget = Robot.DATA.rawLimelight.get(ERawLimelightData.TX_0) > Robot.DATA.rawLimelight.get(ERawLimelightData.TX_1) ? 0 : 1;
                        mIsTracking = true;
                    } else {
                        mSelectedTarget = 0;
                    }
                }

                System.out.println("Selected Target" + mSelectedTarget);
                Robot.DATA.groundTracking.set(ELimelightData.TX, Robot.DATA.rawLimelight.get(getEnumFromString("TX_" + mSelectedTarget)) * (Limelight.llFOVHorizontal / 2));
                Robot.DATA.groundTracking.set(ELimelightData.TY, Robot.DATA.rawLimelight.get(getEnumFromString("TY_" + mSelectedTarget)) * (Limelight.llFOVVertical / 2));
                Robot.DATA.groundTracking.set(ELimelightData.TA, Robot.DATA.rawLimelight.get(getEnumFromString("TA_" + mSelectedTarget)));
                Robot.DATA.groundTracking.set(ELimelightData.TS, Robot.DATA.rawLimelight.get(getEnumFromString("TS_" + mSelectedTarget)));
                Robot.DATA.groundTracking.set(ELimelightData.TL, Robot.DATA.rawLimelight.get(TL));
                Robot.DATA.groundTracking.set(ELimelightData.TSHORT, Robot.DATA.rawLimelight.get(getEnumFromString("TSHORT_" + mSelectedTarget)));
                Robot.DATA.groundTracking.set(ELimelightData.TLONG, Robot.DATA.rawLimelight.get(getEnumFromString("TLONG_" + mSelectedTarget)));
                Robot.DATA.groundTracking.set(ELimelightData.THORIZ, Robot.DATA.rawLimelight.get(getEnumFromString("THORIZ_" + mSelectedTarget)));
                Robot.DATA.groundTracking.set(ELimelightData.TVERT, Robot.DATA.rawLimelight.get(getEnumFromString("TX_" + mSelectedTarget)));
                if (abs(mLastXPosition - Robot.DATA.groundTracking.get(ELimelightData.TX)) > mAcceptableXError || abs(mLastYPosition - Robot.DATA.groundTracking.get(ELimelightData.TY)) > mAcceptableYError) {
                    mIsTracking = false;
                }
                mLastXPosition = Robot.DATA.groundTracking.get(ELimelightData.TX);
                mLastYPosition = Robot.DATA.groundTracking.get(ELimelightData.TY);
            } else {          //set selectedTarget codex straight from limelight codex
            for (ELimelightData e : EnumUtils.getEnums(ELimelightData.class)) {
                Robot.DATA.groundTracking.set(e, Robot.DATA.limelight.get(e));
            }
        }
    }

    /**
     * A small helper function to return an ERawLimelightData from the string representation of that enum.
     * @param pEnum The specified String representation of the desired enum
     * @return The ERawLimelightData enum desired
     */
    private ERawLimelightData getEnumFromString(String pEnum) {
            return Arrays.stream(values()).filter(e -> e.toString().equals(pEnum)).collect(Collectors.toList()).get(0);
        }
}

