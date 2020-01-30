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

    public static Distance getXComponent(Angle pAngle, Distance pHypotenuse) {
        return Distance.fromSI(pHypotenuse.si() * cos(pAngle.radians()));
    }

    public static Distance getYComponent(Angle pAngle, Distance pHypotenuse) {
        return Distance.fromSI(pHypotenuse.si() * sin(pAngle.radians()));
    }

    public static double distance(double x, double y) {
        return distance(0,0,x,y);
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    }

    public static double dotProduct(double x1, double y1, double x2, double y2) {
        return x1 * x2 + y1 * y2;
    }

    public static Angle calculateAngleOffset(Angle pAzimuth, Distance pDistance, Distance pNormalOffset) {
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
