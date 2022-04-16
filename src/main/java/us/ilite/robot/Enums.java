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
        //Angles at desired rung
        NULL(0.0),
        MID(-90.0),
        HIGH(91.0),
        BALANCED(41.0),
        TRAVERSAL(267.5),
        SCORE(250.0);

        //Old traversal angle is 287.5
        final double kAngle;

        EClimberAngle(double pAngle) {
            kAngle = pAngle;
        }
        public double getAngle() {
            return kAngle;
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
        NULL( 0, 0, 0 ),
        DEFAULT( 0, 0, 0 ),
        PURPLE( 125, 0, 250 ),
        RED( 255, 0, 0 ),
        WHITE( 255, 255, 255 ),
        GREEN( 0, 255, 0 ),
        YELLOW( 255, 255, 0 ),
        BLUE( 0, 0, 255 ),
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
        RESET,
        RESET_ODOMETRY,
        PATH_FOLLOWING_BASIC,
        PATH_FOLLOWING_RAMSETE,
        TARGET_ANGLE_LOCK,
        HOLD,
        VELOCITY,
        PERCENT_OUTPUT,
        POSITION,
        SMART_MOTION,
        TURN_TO,
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
        VELOCITY;
    }

}