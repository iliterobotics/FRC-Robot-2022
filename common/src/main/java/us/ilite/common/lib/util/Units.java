package us.ilite.common.lib.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Units {
    public static double rpm_to_rads_per_sec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    public static double rads_per_sec_to_rpm(double rads_per_sec) {
        return rads_per_sec * 60.0 / (2.0 * Math.PI);
    }

    public static double inches_to_meters(double inches) {
        return inches * 0.0254;
    }

    public static double meters_to_inches(double meters) {
        return meters / 0.0254;
    }

    public static double feet_to_meters(double feet) {
        return inches_to_meters(feet * 12.0);
    }

    public static double meters_to_feet(double meters) {
        return meters_to_inches(meters) / 12.0;
    }

    public static double degrees_to_radians(double degrees) {
        return Rotation2d.fromDegrees(degrees).getRadians();
    }

    public static double radians_to_degrees(double radians) {
        return new Rotation2d(radians).getDegrees();
    }

    public static double degrees_to_rotations(double degrees) {
        return degrees / 360.0;
    }

    public static double rotations_to_degrees(double rotations) {
        return rotations * 360.0;
    }

    public static double degrees_to_rotations(double degrees, double smallToBigGearRatio) {
        return degrees_to_rotations(degrees) / smallToBigGearRatio;
    }

    public static double rotations_to_degrees(double rotations, double smallToBigGearRatio) {
        return rotations_to_degrees(rotations) * smallToBigGearRatio;
    }
}
