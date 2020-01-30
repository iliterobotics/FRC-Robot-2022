package us.ilite.common;

import static java.lang.Math.*;


public class Field2020 {

    /**
     * Determines whether the input azimuth falls within the boundary conditions given by the regression trendline
     * a = 0.00006 * x^2 - 0.0241*x + 18.906
     *
     * This was determined emperically via OnShape CAD and has a R^2 value of 0.978 with a safety margin of 0.25 degrees
     *
     * Note that the Azimuth assumes the angle to the INNER GOAL! This is not a direct reading from the LimeLight or other
     * vision system. See us.ilite.common.lib.util.Utils::calculateAngleOffset(...). Also note that it uses distance
     * from the OUTER GOAL, which is also emperically determined via OnShape.
     *
     * @param pAzimuth - Azimuth from the robot to the INNER goal
     * @param pDistance - Distance from the robot to the OUTER goal
     * @return TRUE if the magnitude of the azimuth is less than or equal to the regression line for the given distance
     */
    public static boolean canHitInnerGoal(Angle pAzimuth, Distance pDistance) {
        return abs(pAzimuth.degrees()) <= 0.00006 * pow(pDistance.inches(),2) - 0.0241*pDistance.inches() + 18.906;
    }
}
