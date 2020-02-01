package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum ERawLimelightData implements CodexOf<Double> {
    //  Limelight target values
    TV,         // Whether the limelight has any valid targets (0 or 1)
    TX,         // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    TX_0,        // Raw Screenspace X of contour 0
    TX_1,        // Raw Screenspace X of contour 1
    TX_2,        // Raw Screenspace X of contour 2
    TY,         // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    TY_0,        // Raw Screenspace Y of contour 0
    TY_1,        // Raw Screenspace Y of contour 1
    TY_2,        // Raw Screenspace Y of contour 2
    TA,         // Target Area (0% of image to 100% of image)
    TA_0,        // Target Area of contour 0
    TA_1,        // Target Area of contour 1
    TA_2,        // Target Area of contour 2
    TS,         // Skew or rotation (-90 degrees to 0 degrees)
    TS_0,        // Skew of contour 0
    TS_1,        // Skew of contour 1
    TS_2,        // Skew of contour 2
    TL,         // The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    TSHORT,     // Side-length of shortest side of the fitted bounding box (pixels)
    TSHORT_0,    // Side-length of shortest side of contour 0
    TSHORT_1,    // Side-length of shortest side of contour 1
    TSHORT_2,    // Side-length of shortest side of contour 2
    TLONG,      // Side-length of longest side of the fitted bounding box (pixels)
    TLONG_0,     // Side-length of longest side of contour 0
    TLONG_1,     // Side-length of longest side of contour 1
    TLONG_2,     // Side-length of longest side of contour 2
    THORIZ,     // Horizontal side-length of the rough bounding box (0 - 320 pixels)
    THORIZ_0,    // Horizontal side-length of the rough bounding box of contour 0
    THORIZ_1,    // Horizontal side-length of the rough bounding box of contour 1
    THORIZ_2,    // Horizontal side-length of the rough bounding box of contour 2
    TVERT,      // Vertical side-length of the rough bounding box (0 - 320 pixels)
    TVERT_0,     // Vertical side-length of the rough bounding box of contour 0
    TVERT_1,     // Vertical side-length of the rough bounding box of contour 1
    TVERT_2,     // Vertical side-length of the rough bounding box of contour 2

    //  Limelight LED state
    // 0 - Use Pipeline LED state
    // 1 - Force LED off
    // 2 - Force LED blink
    // 3 - Force LED on
    LED_MODE,

    //  Limelight camera mode
    // 0 - Vision Processor
    // 1 - Driver Camera (Increases exposure, disables vision processing)
    CAM_MODE,    // Current camera mode

    //  Limelight stream mode
    // 0 - Standard - Side-by-side streams if a webcam is attached to Limelight
    // 1 - PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
    // 2 - PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
    STREAM_MODE,    // Current stream mode

    //  Limelight snapshot state
    // 0 - Stop taking snapshots
    // 1 - Take two snapshots per second
    SNAPSHOT_MODE,      // Current snapshot mode

//    CALC_DIST_TO_TARGET,    // The calculated distance to the current target
//    CALC_ANGLE_TO_TARGET,   // The calculated angle to the current target
//    CALC_TARGET_X,          // The calculated x-position of the current target
//    CALC_TARGET_Y,          // The calculated y-position of the current target

    TARGET_ID;

    public enum LedMode {
        NO_CHANGE,
        LED_OFF,
        LED_BLINK,
        LED_ON;
    }

    public enum CamMode {
        VISION_PROCESSOR,
        DRIVER_CAMERA;
    }

    public enum StreamMode {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY;
    }

    public enum SnapshotMode {
        STOP_SNAPSHOTS,
        START_SNAPSHOTS;
    }
}