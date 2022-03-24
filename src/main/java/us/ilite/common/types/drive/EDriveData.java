package us.ilite.common.types.drive;

import com.flybotix.hfr.codex.CodexOf;

public enum EDriveData implements CodexOf<Double> {

    // Sensor inputs
    L_ACTUAL_POS_FT, R_ACTUAL_POS_FT,
    L_DESIRED_POS_FT, R_DESIRED_POS_FT,
    L_ACTUAL_VEL_FT_s, R_ACTUAL_VEL_FT_s,
    L_ACTUAL_VEL_RPM, R_ACTUAL_VEL_RPM,
    LEFT_CURRENT, RIGHT_CURRENT,
    LEFT_VOLTAGE, RIGHT_VOLTAGE,
    DESIRED_LEFT_ft_s, DESIRED_RIGHT_ft_s,
    IS_CURRENT_LIMITING,
    DESIRED_TURN_ANGLE_deg,
    DESIRED_TURN_PCT, DESIRED_THROTTLE_PCT,

    //Ramsete odometry enums
    GET_X_OFFSET, GET_Y_OFFSET,
    GET_X_OFFSET_METERS, GET_Y_OFFSET_METERS,

    ACTUAL_HEADING_RADIANS,
    ACTUAL_HEADING_DEGREES,

    NEUTRAL_MODE,
    STATE,
    L_DESIRED_DRIVE_FT_SEC,
    R_DESIRED_DRIVE_FT_SEC
    ;

}
