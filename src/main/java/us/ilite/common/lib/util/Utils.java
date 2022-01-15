package us.ilite.common.lib.util;

import us.ilite.common.Angle;
import us.ilite.common.Distance;

import static java.lang.Math.*;

public class Utils {

    public static double clamp(double pValue, double pMin, double pMax) {
        return min(pMax, max(pMin, pValue));
    }

    public static double clamp(double pValue, double pMagnitude) {
        return clamp(pValue, -pMagnitude, pMagnitude);
    }

    public static boolean isWithinTolerance(double pValue, double pTarget, double pTolerance) {
        return (pValue - pTolerance <= pTarget) && (pValue + pTolerance >= pTarget);
    }

    /**
     * Determines the x component of the origin-based hypotenuse/angle combo
     * @param pAngle
     * @param pHypotenuse
     * @return
     */
    public static Distance getXComponent(Angle pAngle, Distance pHypotenuse) {
        return Distance.fromSI(pHypotenuse.si() * cos(pAngle.radians()));
    }

    /**
     * Determines the Y component of the origin-based hypotenuse/angle combo
     * @param pAngle
     * @param pHypotenuse
     * @return
     */
    public static Distance getYComponent(Angle pAngle, Distance pHypotenuse) {
        return Distance.fromSI(pHypotenuse.si() * sin(pAngle.radians()));
    }


    /**
     * Calculates a very simple distance of a point from origin. Not fancy.
     *
     * Note that the Y-axis is perpendicular to the field player stations and the X-axis runs parallel to the
     * player stations.
     * @param x
     * @param y
     * @return
     */
    public static double distance(double x, double y) {
        return distance(0,0,x,y);
    }


    /**
     * Calculates a very simple distance between two points where p1 = (x1,y1) and p2 = (x2,y2). Not fancy.
     *
     * Note that the Y-axis is perpendicular to the field player stations and the X-axis runs parallel to the
     * player stations.
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    }

    /**
     * Calculates a very simple dot product between two points where p1 = (x1,y1) and p2 = (x2,y2). Not fancy.
     *
     * Note that the Y-axis is perpendicular to the field player stations and the X-axis runs parallel to the
     * player stations.
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public static double dotProduct(double x1, double y1, double x2, double y2) {
        return x1 * x2 + y1 * y2;
    }

    /**
     * Calculates the angle to a point that is offset on the Y axis by a specific amount. This is useful in years like
     * 2020, when a goal is a linear distance *behind* the vision target.
     *
     * Note that the Y-axis is perpendicular to the field player stations and the X-axis runs parallel to the
     * player stations.
     * @param pAzimuth
     * @param pDistance
     * @param pNormalOffset
     * @return
     */
    public static Angle calculateAngleOffsetY(Angle pAzimuth, Distance pDistance, Distance pNormalOffset) {
        Angle normal = pAzimuth.fromNormal();
        Distance x = getXComponent(normal, pDistance);
        Distance y = getYComponent(normal, pDistance);
        double dA = distance(x.si(), y.si() + pNormalOffset.si());
        double dot = dotProduct(x.si(), y.si(), x.si(), y.si()+ pNormalOffset.si());
        return Angle.fromRadians(
          acos(dot / (pDistance.si() * dA))
        );
    }
}
