package us.ilite.common.types;

public enum EShooterSystemData {
    HOOD_OPEN_LOOP,
    CURRENT_BALL_VELOCITY,      //The Current power cell velocity read from the internal encoders (units?)
    TARGET_BALL_VELOCITY,       //The desired power cell velocity

    CURRENT_FEEDER_VELOCITY_RPM,        //The current feeder velocity read by the internal encoder (units?)
    TARGET_FEEDER_VELOCITY_RPM,         //The desired feeder velocity

    CURRENT_TURRET_ANGLE,           //The current angle of the turret (deg)
    DESIRED_TURRET_ANGLE,           //The desired angle of the turret

    CURRENT_HOOD_ANGLE,             //The current angle of the hood
    TARGET_HOOD_ANGLE,              //The desired angle of the hood

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
