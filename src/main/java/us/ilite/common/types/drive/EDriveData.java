package us.ilite.common.types.drive;

import com.flybotix.hfr.codex.CodexOf;

public enum EDriveData implements CodexOf<Double> {

    // Sensor inputs
    L_ACTUAL_POS_FT, R_ACTUAL_POS_FT,
    L_DESIRED_POS, R_DESIRED_POS,
    L_ACTUAL_VEL_FT_s, R_ACTUAL_VEL_FT_s,
    LEFT_CURRENT, RIGHT_CURRENT,
    LEFT_VOLTAGE, RIGHT_VOLTAGE,
    IS_CURRENT_LIMITING,

    // Outputs from a generated path
    L_PATH_FT_s, R_PATH_FT_s,
    PATH_ERR_ft,

    DESIRED_TURN_PCT, DESIRED_THROTTLE_PCT,
    DESIRED_LEFT_VOLTAGE, DESIRED_RIGHT_VOLTAGE,
    GET_X_OFFSET, GET_Y_OFFSET,
    GET_X_OFFSET_METERS, GET_Y_OFFSET_METERS,

    DELTA_HEADING,
    SET_YAW_RATE_deg_s,
    GYRO_RATE,

    NEUTRAL_MODE,
    STATE,

    ;

}
