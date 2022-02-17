package us.ilite.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

public class Enums {
    // =============================================================================
    // Limelight States
    // =============================================================================
    //  Limelight LED state
    // 0 - Use Pipeline LED state
    // 1 - Force LED off
    // 2 - Force LED blink
    // 3 - Force LED on
    public enum LimelightLedMode {
        NO_CHANGE,
        LED_OFF,
        LED_BLINK,
        LED_ON;
    }

    //  Limelight camera mode
    // 0 - Vision Processor
    // 1 - Driver Camera (Increases exposure, disables vision processing)
    public enum LimelightCamMode {
        VISION_PROCESSOR,
        DRIVER_CAMERA;
    }

    //  Limelight stream mode
    // 0 - Standard - Side-by-side streams if a webcam is attached to Limelight
    // 1 - PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
    // 2 - PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
    public enum LimelightStreamMode {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY;
    }

    //  Limelight snapshot state
    // 0 - Stop taking snapshots
    // 1 - Take two snapshots per second
    public enum LimelightSnapshotMode {
        STOP_SNAPSHOTS,
        START_SNAPSHOTS;
    }

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
        SMART_MOTION,
        TURN_TO,
        TURN_FOR,
        HOME
    }


    // =============================================================================
    // Flywheel States
    // =============================================================================
    public enum FlywheelWheelState {
        NONE,
        OPEN_LOOP,
        VELOCITY
    }

    /**
     * Closed-loop velocity settings for various target speeds
     */
    public enum FlywheelSpeeds {
        OFF(                00.0, 62.0, 0.00, FlywheelWheelState.NONE),
        HOME(               0.0, 30.0, 0.00, FlywheelWheelState.NONE),
        DEFAULT(            20.0, 45.0, 0.75, FlywheelWheelState.VELOCITY),
        CLOSE(              24.0, 62.0, 0.75, FlywheelWheelState.VELOCITY),
        INITIATION_LINE(    29.0, 39.5, 0.75, FlywheelWheelState.VELOCITY),
        FAR(                35.0, 31.5 * 1.1, 0.75, FlywheelWheelState.VELOCITY),
        FAR_TRENCH(         52.0, 22.0 * 1.1, 0.75, FlywheelWheelState.VELOCITY),
        /** Override represents a highly-variable angle / speed state where the controller must directly set everything */
        OVERRIDE(           00.0, 45.0, 0.00, FlywheelWheelState.VELOCITY);

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
        public FlywheelWheelState wheelstate;
        FlywheelSpeeds(double _speed, double _angle, double _feeder, FlywheelWheelState _wheelstate){
            speed=_speed;
            angle=_angle;
            feeder=_feeder;
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
    public enum EArmState {
        NONE (0, 0),
        OUT (90, 1),
        STOW (0, 1),
        HOLD (0, 1);
        public double angle;
        public int slot;
        EArmState (double angle, int slot) {
            this.angle = angle;
        }
    }

}
