package us.ilite.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.robot.modules.LEDControl;

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

    // LED control and colors
    public enum LEDColorMode {
        PURPLE( 255, 0, 200 ),
        RED( 255, 0, 0 ),
        LIGHT_BLUE( 0, 100, 220 ),
        WHITE( 255, 255, 255 ),
        GREEN( 0, 255, 0 ),
        YELLOW( 255, 255, 0 ),
        GREEN_HSV( 84, 255, 255 ),
        BLUE( 0, 0, 255 ),
        RED_HSV( 0, 255, 255 ),
        YELLOW_HSV( 20, 255, 255 ),
        PURPLE_HSV( 212, 255, 255 ),
        ORANGE( 255, 165, 0 ),
        DEFAULT( 0, 0, 0 );
        double red;
        double green;
        double blue;
        LEDColorMode(double pR, double pG, double pB) {
            red = pR;
            green = pG;
            blue = pB;
        }

        public double getRed() {
            return red;
        }

//        Color clr = new Color(red, green, blue);
//
//        public Color getColor() {
//            return this.clr;
//        }
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
        POSITION,
        MOTION_MAGIC,
        TURN_TO,
        TURN_FOR,
        HOME,
        TANK
    }



    // =============================================================================
    // Intake Module States
    // =============================================================================
    public enum EIntakeState {
        PERCENT_OUTPUT,
        VELOCITY;
    }

    public enum EArmState {
        EXTEND,
        RETRACT,
        DEFAULT;
    }
    public enum EFeederState {
        PERCENT_OUTPUT,
        INDEXING_VELOCITY;
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
