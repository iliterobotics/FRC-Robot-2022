package us.ilite.common;

import static java.lang.Math.*;

public class Angle {

    private final double mRadians;

    private Angle(double pRadians) {
        mRadians = pRadians;
    }

    public Angle add(Angle pOther) {
        return fromRadians(mRadians + pOther.mRadians);
    }

    public Angle subtract(Angle pOther) {
        return fromRadians(mRadians - pOther.mRadians);
    }

    public double radians() {
        return mRadians;
    }

    public double degrees() {
        return toDegrees(mRadians);
    }

    public Angle fromNormal() {
        return fromRadians(PI/2 - mRadians);
    }

    public static Angle fromDegrees(double pDegrees) {
        return fromRadians(toRadians(pDegrees));
    }

    public static Angle fromRadians(double pRadians) {
        return new Angle(pRadians);
    }
}
