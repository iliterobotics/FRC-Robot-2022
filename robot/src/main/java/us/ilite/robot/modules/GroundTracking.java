package us.ilite.robot.modules;

import com.flybotix.hfr.util.lang.EnumUtils;
import us.ilite.common.types.EVisionGoal2020;

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
    public void readInputs() {
        sortTrackingData();
    }

    @Override
    public void setOutputs() {
    }

    public void sortTrackingData() {
        Robot.DATA.groundTracking.set(EVisionGoal2020.TV, Robot.DATA.rawLimelight.get(ERawLimelightData.TV));
        boolean targetValid = Robot.DATA.groundTracking.isSet(EVisionGoal2020.TV);
        if (targetValid) {
            if (!mIsTracking) {
                if (abs(Robot.DATA.rawLimelight.get(ERawLimelightData.TY_0) - Robot.DATA.rawLimelight.get(ERawLimelightData.TY_1)) < mAcceptableError) {
                    mSelectedTarget = Robot.DATA.rawLimelight.get(ERawLimelightData.TX_0) > Robot.DATA.rawLimelight.get(ERawLimelightData.TX_1) ? 0 : 1;
                    mIsTracking = true;
                } else {
                    mSelectedTarget = 0;
                }
            }

            Robot.DATA.groundTracking.set(EVisionGoal2020.TX, Robot.DATA.rawLimelight.get(getEnumFromString("TX_" + mSelectedTarget)) * (Limelight.llFOVHorizontal / 2));
            Robot.DATA.groundTracking.set(EVisionGoal2020.TY, Robot.DATA.rawLimelight.get(getEnumFromString("TY_" + mSelectedTarget)) * (Limelight.llFOVVertical / 2));
            Robot.DATA.groundTracking.set(EVisionGoal2020.TS, Robot.DATA.rawLimelight.get(getEnumFromString("TS_" + mSelectedTarget)));
            Robot.DATA.groundTracking.set(EVisionGoal2020.TL, Robot.DATA.rawLimelight.get(TL));
            if (abs(mLastXPosition - Robot.DATA.groundTracking.get(EVisionGoal2020.TX)) > mAcceptableXError || abs(mLastYPosition - Robot.DATA.groundTracking.get(EVisionGoal2020.TY)) > mAcceptableYError) {
                mIsTracking = false;
            }
            mLastXPosition = Robot.DATA.groundTracking.get(EVisionGoal2020.TX);
            mLastYPosition = Robot.DATA.groundTracking.get(EVisionGoal2020.TY);
        } else {          //set selectedTarget codex straight from limelight codex
            for (EVisionGoal2020 e : EnumUtils.getEnums(EVisionGoal2020.class)) {
                Robot.DATA.groundTracking.set(e, Robot.DATA.goaltracking.get(e));
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
