
package us.ilite.common.types;


import com.flybotix.hfr.codex.CodexOf;

public enum ELimelightData implements CodexOf<Double> {
    ledmode,    // Sets limelight’s LED state
    tv,         // Whether the limelight has any valid targets (0 or 1)
    tx,         // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    ty,         // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    ta,         // Target Area (0% of image to 100% of image)
    ts,         // Skew or rotation (-90 degrees to 0 degrees)
    tl,         // The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    tshort,     // Sidelength of shortest side of the fitted bounding box (pixels)
    tlong,      // Sidelength of longest side of the fitted bounding box (pixels)
    thoriz,     // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    tvert,      // Vertical sidelength of the rough bounding box (0 - 320 pixels)
    
    targetOrdinal,
    calcDistToTarget,
    calcAngleToTarget,
    calcTargetX,
    calcTargetY,

    TRACKING_TYPE;

}