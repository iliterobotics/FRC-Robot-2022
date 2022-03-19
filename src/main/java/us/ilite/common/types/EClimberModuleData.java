package us.ilite.common.types;

public enum EClimberModuleData {
    // Actual
    L_VEL_rpm,R_VEL_rpm,
    L_POSITION_deg,R_POSITION_deg,
    L_OUTPUT_CURRENT,R_OUTPUT_CURRENT,
    BUS_VOLTAGE_RIGHT, BUS_VOLTAGE_LEFT,

    // Desired
    DESIRED_VEL_rpm,
    DESIRED_VEL_deg_s,
    DESIRED_POS_deg,
    DESIRED_VEL_pct,

    // States
    HANGER_STATE,
    DOUBLE_CLAMPED,
    SINGLE_CLAMPED,
    SET_COAST,
    CURRENT_RUNG,
    CURRENT_STAGE,
    DESIRED_RUNG,
    RUNG_CAPTURED
    }
