package us.ilite.common.lib.util;

public class Utils {

    public static double clamp(double pValue, double pMin, double pMax) {
        return Math.min(pMax, Math.max(pMin, pValue));
    }

    public static double clamp(double pValue, double pMagnitude) {
        return clamp(pValue, -pMagnitude, pMagnitude);
    }

    public static boolean isWithinTolerance(double pValue, double pTarget, double pTolerance) {
        return (pValue - pTolerance <= pTarget) && (pValue + pTolerance >= pTarget);
    }
}
