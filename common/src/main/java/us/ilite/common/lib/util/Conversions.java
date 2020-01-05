package us.ilite.common.lib.util;

import us.ilite.common.config.Settings;

public class Conversions {
    
    public static double rotationsToInches(double rotations) {
        return rotations * (Settings.Drive.kWheelDiameterInches * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / Settings.Drive.kWheelCircumference;
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * Settings.Drive.kTicksPerRotation / 10.0;
    }

    public static double ticksToRotations(double ticks) {
        return ticks / Settings.Drive.kTicksPerRotation;
    }

    public static double ticksToInches(double ticks) {
        return ticksToRotations(ticks) * Settings.Drive.kWheelCircumference;
    }

    public static int inchesToTicks(double inches) {
        return (int)(inchesToRotations(inches) * Settings.Drive.kTicksPerRotation);
    }

    public static double ticksPer100msToRotationsPerSecond(double ticks) {
        return ticks / Settings.Drive.kTicksPerRotation * 10.0;
    }

    public static double ticksPer100msToInchesPerSecond(double ticks) {
        return ticksPer100msToRotationsPerSecond(ticks) * Settings.Drive.kWheelCircumference;
    }

    public static double ticksPer100msToRadiansPerSecond(double ticks) {
        return ticksPer100msToRotationsPerSecond(ticks) * (Math.PI * 2.0);
    }
    
}
