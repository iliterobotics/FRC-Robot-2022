package us.ilite.common.lib.util;

/**
 * @author jmshapiro
 * RangeScale will scale an input value in range a to range b or the reverse.
 */
public class RangeScale {

    private double rangeAMin;
    private double rangeAMax;
    private double rangeBMin;
    private double rangeBMax;

    public RangeScale(double rangeAMin, double rangeAMax, double rangeBMin, double rangeBMax ) {
        this.rangeAMin = rangeAMin;
        this.rangeAMax = rangeAMax;
        this.rangeBMin = rangeBMin;
        this.rangeBMax = rangeBMax;
    }

    public double scaleAtoB(double aInput) {
        return (aInput - this.rangeAMin) / (this.rangeAMax - this.rangeAMin) * (this.rangeBMax - this.rangeBMin) + this.rangeBMin;
    }

    public double scaleBtoA(double bInput) {
        return (bInput - this.rangeBMin) / (this.rangeBMax - this.rangeBMin) * (this.rangeAMax - this.rangeAMin) + this.rangeAMin;
    }
}
