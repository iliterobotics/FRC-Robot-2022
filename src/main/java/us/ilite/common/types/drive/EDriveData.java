package us.ilite.common.types.drive;

import com.flybotix.hfr.codex.CodexOf;

public enum EDriveData implements CodexOf<Double> {

    // Sensor inputs
    L_ACTUAL_POS_FT, R_ACTUAL_POS_FT,
    L_DESIRED_POS, R_DESIRED_POS,
    L_ACTUAL_VEL_FT_s, R_ACTUAL_VEL_FT_s,
    L_ACTUAL_VEL_RPM, R_ACTUAL_VEL_RPM,
    L_DESIRED_VEL_RPM, R_DESIRED_VEL_RPM,
    LEFT_CURRENT, RIGHT_CURRENT,
    LEFT_VOLTAGE, RIGHT_VOLTAGE,
    DESIRED_LEFT_VOLTAGE, DESIRED_RIGHT_VOLTAGE,
    IS_CURRENT_LIMITING,

    DESIRED_TURN_ANGLE_deg,
    ACTUAL_TURN_ANGLE_deg,

    // Outputs from a generated path
    L_PATH_FT_s, R_PATH_FT_s,
    PATH_ERR_ft,

    DESIRED_TURN_PCT, DESIRED_THROTTLE_PCT,
    DESIRED_LEFT_PCT, DESIRED_RIGHT_PCT,
    ACTUAL_LEFT_PCT, ACTUAL_RIGHT_PCT,

    DELTA_HEADING,
    SET_YAW_RATE_deg_s,
    GYRO_RATE,

    NEUTRAL_MODE,
    STATE,
    ;

}
