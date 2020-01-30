package us.ilite.common;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class Distance implements Science {
    // meters
    private final double si;

    public static NumberFormat nf = new DecimalFormat("0.00");

    public static double FEET_PER_METER = 0.3048;
    public static double INCHES_PER_FEET = 12;

    public double si() { return si; }
    public double meters() { return si; }
    public double feet() { return si * FEET_PER_METER; }
    public double inches() { return feet() * 12; }

    private Distance(double si) {this.si = si;}
    public static Distance fromSI(double si) { return new Distance(si);}
    public static Distance fromMeters(double meters) { return fromSI(meters);}
    public static Distance fromFeet(double feet) { return fromMeters(feet/ FEET_PER_METER);}
    public static Distance fromInches(double inches) { return fromFeet(inches / INCHES_PER_FEET);}

    public String toString() {
        return nf.format(si);
    }
}
