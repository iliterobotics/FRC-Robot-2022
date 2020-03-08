package us.ilite.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

public class Enums {
    // =============================================================================
    // Drivetrain States
    // =============================================================================
    public enum EDriveState {
        NORMAL,
        RESET,
        PATH_FOLLOWING_BASIC,
        PATH_FOLLOWING_HELIX,
        PATH_FOLLOWING_RAMSETE,
        TARGET_ANGLE_LOCK,
        HOLD,
        VELOCITY,
        PERCENT_OUTPUT,
        SMART_MOTION
    }

    // =============================================================================
    // DJ Spinner States
    // =============================================================================
    public enum EColorMatch {
        NONE(0, 0, 0 ),
        RED( 0.561, 0.232, 0.114 ),
        YELLOW( 0.361, 0.524, 0.113 ),
        BLUE(  0.143, 0.427, 0.429 ),
        GREEN(  0.197, 0.561, 0.240 );
        public Color color;
        public double r;
        public double g;
        public double b;

        EColorMatch(double r, double g, double b) {
            color = new Color(r,g,b);
        }

        public Color getColorMatch() {
            return ColorMatch.makeColor(r, g, b);
        }

        public EColorMatch nextColor() {
            if(this.ordinal() == GREEN.ordinal()) {
                return RED;
            } else if (this.ordinal() == NONE.ordinal() ){
                return NONE;
            }
            else {
                return EColorMatch.values()[ordinal()+1];
            }
        }

        public static EColorMatch from(ColorMatchResult pResult) {
            for(EColorMatch cm : values()) {
                if(equal(cm.color, pResult.color)) {
                    return cm;
                }
            }
            return NONE;
        }

        private static boolean equal(Color a, Color b) {
            //TODO - this is where we put the thresholding for different color
            // e.g. if abs(a.red - b.red) <= 0.1 then they are close
            return a.equals(b);
        }
    }

    public enum EColorWheelState {
        OFF (0.0),
        ROTATION (0.4),
        POSITION (0.2);

        public double power;
        private EColorWheelState(double _power) {
            power = _power;
        }

        public double getPower(){
            return this.power;
        }
        public static EColorWheelState valueOf(double pOrdinal) {
            return values()[(int)pOrdinal];
        }
    }



    // =============================================================================
    // Flywheel States
    // =============================================================================
    public enum HoodState {
        NONE,
        TARGET_ANGLE,
        MANUAL
    }
    public enum EHoodSensorError {
        NONE,
        INVALID_POTENTIOMETER_READING
    }
    public enum FlywheelWheelState {
        NONE,
        OPEN_LOOP,
        VELOCITY
    }
    public enum TurretControlType {
        MANUAL,
        TARGET_LOCKING,
        HOME
    }

    /**
     * Closed-loop velocity settings for various target speeds
     */
    public enum FlywheelSpeeds {
        OFF(                00.0, 62.0, 0.00, HoodState.NONE, FlywheelWheelState.NONE),
        HOME(               0.0, 30.0, 0.00, HoodState.TARGET_ANGLE, FlywheelWheelState.NONE),
        DEFAULT(            20.0, 45.0, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        CLOSE(              24.0, 62.0, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        INITIATION_LINE(    29.0, 39.5, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        FAR(                35.0, 31.5, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        FAR_TRENCH(         52.0, 22.0, 0.75, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY),
        /** Override represents a highly-variable angle / speed state where the controller must directly set everything */
        OVERRIDE(           00.0, 45.0, 0.00, HoodState.TARGET_ANGLE, FlywheelWheelState.VELOCITY);

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



    // =============================================================================
    // Power Cell Module States
    // =============================================================================
    public enum EIntakeState {
        //TODO find velocities
        STOP (0.0),
        INTAKE (0.1),
        REVERSE (-0.1);

        double pPower;

        EIntakeState (double pPower) {
            this.pPower = pPower;
        }

        public double getPower() {
            return pPower;
        }
    }

    public enum EArmState {
        NONE(0.0, 0),
        OUT(90.0, 1),
        // TODO - fix to UP slot
        STOW(0.0, 1),
        HOLD(0.0, 1);

        public double angle;
        public int slot;

        EArmState (double angle, int slot) {
            this.angle = angle;
        }
    }

    public enum EIndexingState {
        BROKEN (true),
        NOT_BROKEN(false);

        boolean broken;

        EIndexingState(boolean broken) {
            this.broken = broken;
        }

        public boolean getState(){
            return this.broken;
        }
    }

    // =============================================================================
    // Power Cell Module States
    // =============================================================================
    public enum EHangerControlState {
        HOLD,
        MOVE
    }
}
