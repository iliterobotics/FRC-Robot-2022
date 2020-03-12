package us.ilite.robot.vision;

import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

public class CameraConfig {

    public static final CameraConfig
            LIMELIGHT_V2_LOW_RES = new CameraConfig(49.7, 59.6, 90.0, 1000.0/90.0, 320, 240),
            LIMELIGHT_V2_HI_RES = new CameraConfig(49.7, 59.6, 90.0, 1000.0/90.0, 0d, 0d)
    ;

    public final double
            hResolution,
            hFOV_deg,
            vResolution,
            vFOV_deg,
            framerate,
            latency,

            // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
            vpw,
            vph
        ;
    private double lensheight = 0;
    private double elevationAngle = 0;
    private boolean isInverted = false;
    public String address = "";


    public CameraConfig(double _vFOV_deg, double _hFOV_deg, double _framerate, double _latency, double _hResolution, double _vResolution) {
        vFOV_deg = _vFOV_deg;
        hFOV_deg = _hFOV_deg;
        hResolution = _hResolution;
        vResolution = _vResolution;
        framerate = _framerate;
        latency = _latency;

        // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
        vpw = 2.0*tan(toRadians(hFOV_deg)/2);
        vph = 2.0*tan(toRadians(vFOV_deg)/2);
    }

    public double lensheight_in() {
        return lensheight;
    }

    public double elevation_deg() {
        return elevationAngle;
    }

    public boolean isInverted() { return isInverted; }

    /**
     * Updates the floor height of the camera
     * @param pElevationAngle_deg elevation angle from the floor in degrees, range = [-90,90]. 0 = Camera lens is
     *                        perfectly level. 90 = Camera lens faces the ceiling; -90 = Camera lens faces the floor. In
     *                        some configurations this changes often (such as when it's on a servo).
     * @return the config object, to support the builder pattern
     */
    public CameraConfig setElevationAngle(double pElevationAngle_deg) {
        elevationAngle = pElevationAngle_deg;
        return this;
    }

    /**
     * Updates the floor height of the camera
     * @param pLensHeight_in height from the floor, in inches
     * @return the config object, to support the builder pattern
     */
    public CameraConfig setLensHeight(double pLensHeight_in) {
        lensheight = pLensHeight_in;
        return this;
    }

    /**
     * Turns the camera upside down
     * @param pIsInverted TRUE if camera is inverted. False otherwise.
     * @return the config object, to support the builder pattern
     */
    public CameraConfig setInverted(boolean pIsInverted) {
        isInverted = pIsInverted;
        return this;
    }

    /**
     * Update the address (IP, networktable, etc) of this camera
     * @param pAddress - format is implementation-specific
     * @return the config object, to support the builder pattern
     */
    public CameraConfig setAddress(String pAddress) {
        address = pAddress;
        return this;
    }
}
