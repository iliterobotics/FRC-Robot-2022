package us.ilite.robot;

public class Enums {

    public enum HoodState {
        NONE,
        TARGET_ANGLE,
        MANUAL
    }


    public enum FlywheelWheelState {
        NONE,
        OPEN_LOOP,
        VELOCITY,
    }

    public enum FlywheelSpeeds {
        OFF(0.0, 62.0, 0.0, HoodState.NONE, FlywheelWheelState.NONE),
        DEFAULT(20.0, 45.0, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        CLOSE(20.0, 43.6, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        INITIATION_LINE(28.0, 39.5, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        FAR(33.7, 31.5, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        FAR_TRENCH(50.0, 22.0, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY);
        /**
         * Ball trajectory exit speed in feet per second
         */
        public double speed;
        /**
         * Ball trajectory exit angle from the floor, in degrees
         */
        public double angle;
        /**
         * Feeder wheel PCT_OUTPUT, {-1,1}
         */
        public double feeder;
        public HoodState hoodstate;
        public FlywheelWheelState wheelstate;
        FlywheelSpeeds(double _speed, double _angle, double _feeder, HoodState _hoodstate, FlywheelWheelState _wheelstate){
            speed=_speed;
            angle=_angle;
            feeder=_feeder;
            hoodstate = _hoodstate;
            wheelstate = _wheelstate;
        }
    }
}
