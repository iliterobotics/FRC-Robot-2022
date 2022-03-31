package us.ilite.robot;

public class Enums {
    public enum EClimberMode {
        VELOCITY,
        POSITION,
        PERCENT_OUTPUT,
        BEGIN_HANG,
        DEFAULT
    }

    public enum EClampMode {
        NULL,
        CLAMPED,
        RELEASED
    }

    public enum EClimberAngle {
        // Rungs/Stages
        VERTICAL(90, 0),
        MID(90, 1),
        HIGH(-15, 2),
        TRAVERSAL(-195, 3),

        // States
        START(0),
        END(45),
        BALANCE(0);

        final int kAngle;
        final int kStage;

        EClimberAngle(int pAngle) {
            kAngle = pAngle;
            kStage = -1;
        }

        EClimberAngle(int pAngle, int pStage) {
            kAngle = pAngle;
            kStage = pStage;
        }

        public int getAngle() {
            return kAngle;
        }

        public int getStage() {
            return kStage;
        }
    }

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
        DEFAULT( 0, 0, 0 ),
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
        ORANGE( 255, 165, 0 );
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
        public double getGreen() {
            return green;
        }
        public double getBlue() {
            return blue;
        }


//        Color clr = new Color(red, green, blue);
//
//        public Color getColor() {
//            return this.clr;
//        }
    }

    public enum LEDState {
        NULL,
        BLINKING,
        SOLID;
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
        NULL,
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
        SMART_MOTION,
        TURN_TO,
        TURN_FOR,
        HOME,
        TANK
    }



    // =============================================================================
    // Intake Module States
    // =============================================================================
    public enum ERollerState {
        NULL,
        PERCENT_OUTPUT,
        VELOCITY;
    }

    public enum EArmState {
        NULL,
        EXTEND,
        RETRACT,
        DEFAULT;
    }
    public enum EFeederState {
        NULL,
        PERCENT_OUTPUT,
        INDEXING_VELOCITY;
    }

}