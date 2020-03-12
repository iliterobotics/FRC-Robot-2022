package us.ilite.common.types;

public enum EVisionGoal2020 {
    //  Limelight target values
    TV,         // Whether the limelight has any valid targets (0 or 1)
    TX,         // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    TY,         // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    TS,         // Skew or rotation (-90 degrees to 0 degrees)
    TL,         // The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.

    PIPELINE,   // Current vision targeting pipeline
    LED_MODE,
    CAM_MODE,    // Current camera mode
    STREAM_MODE,    // Current stream mode
    SNAPSHOT_MODE,      // Current snapshot mode

    TARGET_RANGE_in,    // The calculated distance to the current target
    TARGET_AZIMUTH_deg,   // The calculated angle to the current target
    T3D_TOP_X_in,          // The calculated x-position of the current target
    T3D_TOP_Y_in,          // The calculated y-position of the current target
    T3D_TOP_GOAL_RANGE_in,
    T3D_TOP_LEFT_RANGE_in,
    T3D_TOP_RIGHT_RANGE_in,
    T3D_TOP_AZIMUTH_deg,
    T3D_TOP_AZ_OFFSET_deg,
    T3D_BOT_X_in,          // The calculated x-position of the current target
    T3D_BOT_Y_in,          // The calculated y-position of the current target
    T3D_BOT_GOAL_RANGE_in,
    T3D_BOT_LEFT_RANGE_in,
    T3D_BOT_RIGHT_RANGE_in,
    T3D_BOT_AZIMUTH_deg,
    T3D_BOT_AZ_OFFSET_deg,
    TARGET_ID,          // If this is not set, then the Limelight is not tracking

    T_LEFT_X,
    T_LEFT_Y,
    T_RIGHT_X,
    T_RIGHT_Y,

    ANGLE_FROM_HORIZON;
}