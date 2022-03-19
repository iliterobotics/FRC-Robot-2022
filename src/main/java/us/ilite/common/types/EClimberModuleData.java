package us.ilite.common.types;

public enum EClimberModuleData {
    // Actual
    L_VEL_rpm,
//    R_VEL_rpm,
    L_POSITION_deg,
//    R_POSITION_deg,
    L_OUTPUT_CURRENT,
    R_OUTPUT_CURRENT,
//    BUS_VOLTAGE_RIGHT,
    BUS_VOLTAGE_LEFT,
    L_POSITION_TARGET,
//    R_POSITION_TARGET,
    L_POSITION_ERROR,
//    R_POSITION_ERROR,

    // Desired
    DESIRED_VEL_rpm,
    DESIRED_POS_deg,
    DESIRED_VEL_pct,

    // States
    HANGER_STATE,
    IS_DOUBLE_CLAMPED,
    IS_SINGLE_CLAMPED,
    SET_COAST,
//    CURRENT_RUNG,
//    CURRENT_STAGE,
//    DESIRED_RUNG,
//    RUNG_CAPTURED
    }
