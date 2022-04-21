package us.ilite.common.types;

public enum EClimberData {
    /**Actual left velocity in rpm**/
    ACTUAL_VEL_rpm,
    SINGLE_BEAM_BROKEN,

    /**Actual left position in degrees**/
    ACTUAL_POSITION_deg,

    /**Left output in current**/
    ACTUAL_OUTPUT_CURRENT_12,
    ACTUAL_OUTPUT_CURRENT_11,


    /**Bus voltage left**/
    ACTUAL_BUS_VOLTAGE,

    /**Left position target**/
    ACTUAL_POSITION_TARGET,

    /**Left position error**/
    ACTUAL_POSITION_ERROR,

    /**Desired velocity in rpm**/
    DESIRED_VEL_rpm,

    /**Desired position in degrees**/
    DESIRED_POS_deg,

    /**Desired velocity in percent of maximum velocity**/
    DESIRED_VEL_pct,

    /**State of the hanger**/
    HANGER_STATE,

    /**State of the double-sided clamp**/
    IS_DOUBLE_CLAMPED,

    /**State of the single-sided clamp**/
    IS_SINGLE_CLAMPED,

    /**State if the coast is set**/
    SET_COAST,

    /**Applied velocity in percent of maximum velocity**/
    ACTUAL_CLIMBER_PCT,

    // ========================== //
    // ====== Experimental ====== //
    // ========================== //

    DESIRED_RUNG,
    CURRENT_RUNG,
    STAGE,
    RUNG_STATE,
    }