package us.ilite.common.types;

public enum EClimberModuleData {
    /**Actual left velocity in rpm**/
    L_VEL_rpm,
    /**Actual left position in degrees**/
    L_POSITION_deg,
    /**Right Position in degrees**/
    R_POSITION_deg,
    /**Left output in current**/
    L_OUTPUT_CURRENT,
    /**Right output in current**/
    R_OUTPUT_CURRENT,
    /**Bus voltage left**/
    BUS_VOLTAGE_LEFT,
    /**Left position target**/
    L_POSITION_TARGET,
    /**Left position error**/
    L_POSITION_ERROR,
    /**Desired velocity in rpm**/
    DESIRED_VEL_rpm,
    /**Desired position in degrees**/
    DESIRED_POS_deg,
    /**Desired velocity in percent of maximum velocity**/
    DESIRED_VEL_pct,
    /**State of the hanger**/
    HANGER_STATE,
    /**State if the double clamp is clamped**/
    IS_DOUBLE_CLAMPED,
    /**State if the single clamp is clamped**/
    IS_SINGLE_CLAMPED,
    /**State if the coast is set**/
    SET_COAST,
    }