package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EFeederData implements CodexOf<Double>{
//    FLYWHEEL_RAW_SPEED,
//    BALL_VELOCITY_ft_s,      //The Current power cell velocity read from the internal encoders (units?)
//    SET_BALL_VELOCITY_ft_s,       //The desired power cell velocity
//
//    FEEDER_rpm,        //The current feeder velocity read by the internal encoder (units?)
//    SET_FEEDER_rpm,         //The desired feeder velocity
//    SET_FEEDER_pct,
//
//    TURRET_ANGLE_deg,           //The current angle of the turret (deg)
//    SET_TURRET_ANGLE_deg,           //The desired angle of the turret
//    MANUAL_TURRET_DIRECTION,
//    TURRET_CONTROL,
//    HOME_REVERSED,
//    IS_TARGET_LOCKED,
//
//    FLYWHEEL_SPEED_STATE,
//
//    HOOD_STATE,
//    HOOD_ANGLE_deg,             //The current angle of the hood
//    SET_HOOD_ANGLE_deg,             //The desired angle of the hood
//    SET_HOOD_pct,HOOD_SERVO_RAW_VALUE,
//    HOOD_SENSOR_ERROR,
//    POT_RAW_VALUE,POT_AVG_VALUE,
//
//    FLYWHEEL_WHEEL_STATE,
//    FLYWHEEL_OPEN_LOOP

    ENTRY_BEAM,
    EXIT_BEAM,
    NUM_BALLS,
    STATE,

    SET_CONVEYOR_pct,
    CONVEYOR_pct,
}
