package us.ilite.common.types;

public enum EShooterSystemData {
    HOOD_OPEN_LOOP,
    SET_BALL_VELOCITY_ft_s,      //The Current power cell velocity read from the internal encoders (units?)
    BALL_VELOCITY_ft_s,       //The desired power cell velocity

    FEEDER_rpm,        //The current feeder velocity read by the internal encoder (units?)
    SET_FEEDER_rpm,         //The desired feeder velocity

    CURRENT_FEEDER_VELOCITY,
    CURRENT_TURRET_ANGLE,           //The current angle of the turret (deg)
    DESIRED_TURRET_ANGLE,           //The desired angle of the turret
    MANUAL_TURRET_DIRECTION,
    TURRET_CONTROL,
    HOME_REVERSED,
    IS_TARGET_LOCKED,

    CURRENT_HOOD_ANGLE,             //The current angle of the hood
    TARGET_HOOD_ANGLE,             //The desired angle of the hood

    FEEDER_OUTPUT_OPEN_LOOP,
    FLYWHEEL_SPEED_STATE,

    HOOD_STATE,
    HOOD_SENSOR_ERROR,
    HOOD_SERVO_RAW_VALUE, // TODO - prune
    HOOD_SERVO_LAST_VALUE, // TODO - prune
    POT_NORM_VALUE,
    POT_AVG_VALUE,
    POT_RAW_VALUE, // TODO - prune

    FLYWHEEL_WHEEL_STATE,
    FLYWHEEL_IS_MAX_VELOCITY,
    FLYWHEEL_DISTANCE_BASED_SPEED,
    FLYWHEEL_OPEN_LOOP
}
