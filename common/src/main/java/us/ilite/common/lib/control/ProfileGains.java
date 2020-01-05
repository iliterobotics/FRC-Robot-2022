package us.ilite.common.lib.control;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import us.ilite.common.config.Settings;

/**
 * A data structure meant to hold PIDF, Max Accel, Max Speed, etc.
 * No assumption are made about units.
 * All values are defaulted to zero.
 */
public class ProfileGains {
    public double P = 0;
    public double I = 0;
    public double D = 0;
    public double F = 0;

    public double MAX_ACCEL = 0;
    public double MAX_VELOCITY = 0;
    public double TOLERANCE = 0;

    /** Defaulted to 1 */
    public int PROFILE_SLOT = 1;

    public TrapezoidProfile.Constraints generateConstraints() {
        return new TrapezoidProfile.Constraints(MAX_VELOCITY,MAX_ACCEL);
    }

    public ProfiledPIDController generateController() {
        ProfiledPIDController controller = new ProfiledPIDController(P, I, D, generateConstraints(), Settings.kControlLoopPeriod);
        controller.setTolerance(TOLERANCE);
        return controller;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains p(double gain) {
        P = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains i(double gain) {
        I = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains d(double gain) {
        D = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains f(double gain) {
        F = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains maxAccel(double gain) {
        MAX_ACCEL = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains maxVelocity(double gain) {
        MAX_VELOCITY = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains tolerance(double gain) {
        TOLERANCE = gain;
        return this;
    }

    /**
     * Builder-pattern helper for constructing
     * @param gain
     * @return
     */
    public ProfileGains slot(int gain) {
        PROFILE_SLOT = gain;
        return this;
    }
}