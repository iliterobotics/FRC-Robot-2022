package us.ilite.common.config;

import us.ilite.common.lib.control.ProfileGains;

/**
 * ONLY ROBOT-wide settings should go into this class (e.g. things like
 *  - CAN/DIO/etc ID's
 *  - IP ports, addresses
 *  - Driver Input
 *  - Field element locations & vision target heights
 */
public class Settings {
    public static final String kGroundLimelightNetworkTable = "limelight-bottom";
    public static final String kFlywheelLimelightNetworkTable = "limelight-top";


    public static double kControlLoopPeriod = 0.02; // seconds

    public static int kSecondsToUpdateCSVLogger = 1; //seconds
    public static int kAcceptableLogFailures = 8;
    public static double kNetworkTableUpdateRate = 0.01;
    public static boolean kIsLogging = true; // decide whether or not to log

    public static int sCODEX_COMMS_PORT = 5805;

    public static final String AUTO_PATH_PACKAGE = "us.ilite.robot.auto.paths";

    public static final String CONTROLLER_PATH_PACKAGE = "us.ilite.robot.controller";

    // ==================================================
    // System ID's
    // DO NOT CHANGE ANY OF THE FOLLOWING VALUES. PERIOD.
    // ALWAYS CONSULT WITH MENTORS OR LEADERSHIP
    // ==================================================
    public static class HW {
        public static class CAN {
            public static int kTimeoutMs = 10; //use for on the fly updates
            public static int kLongTimeoutMs = 100; //use for constructors

            // ===============
            // DRIVETRAIN ID's
            // ===============
            public static final int kDTMR2 = 2;
            public static final int kDTR4 = 4;
            public static final int kDTML1 = 1;
            public static final int kDTML3 = 3;
            public static final int kDTGyro = 30;

            // ======================
            // FEEDER AND INTAKE ID's
            // ======================
            public static final int kINRoller = 9;
            public static final int kINFeeder = 10;

            // ============
            // CLIMBER ID's
            // ============
            public static final int kCLM1 = 11;
            public static final int kCL2 = 12;

            // ========
            // LED ID's
            // ========
            public static int kLEDCanifier = 31;
        }

        public static class Analog {
        }

        public static class DIO {
            public static final int kEDTMR2 = 3;
            public static final int kEDTML1 = 1;
            public static final int kINEntryBeam = 9;
            public static final int kINExitBeam = 10;
        }

        public static class PCH {
            // ======================
            // FEEDER AND INTAKE ID's
            // ======================
            public static final int kINPNIntakeForward = 1;
            public static final int kINPNIntakeReverse = 2;
            public static final int kINPNFeeder = 3;

            // ============
            // CLIMBER ID's
            // ============
            public static final int kCLPNClimbForward = 4;
            public static final int kCLPNClimbReverse = 5;
        }

    }

    public static class Input {

        public static double kNormalPercentThrottleReduction = 1.0;
        // These are applied AFTER the normal throttle reduction
        public static double kSnailModePercentThrottleReduction = 0.5;
        public static double kSnailModePercentRotateReduction = 0.4;
        public static double kMaxAllowedVelocityMultiplier = 0.75;

        // Applied after any scaling
        public static double kDriverInputTurnMaxMagnitude = 0.5;

        public static double kInputDeadbandF310Joystick = 0.05;
        public static double kInputDeadbandF310Trigger = 0.5;
        public static int kJoystickPortDriver = 0;
        public static int kJoystickPortOperator = 1;
        public static int kJoystickPortTester = 2;
    }

    // =============================================================================
    // PID TargetLock constants
    // =============================================================================
    public static ProfileGains kTargetAngleLockGains = new ProfileGains().p(0.0005);
    public static ProfileGains kTargetDistanceLockGains = new ProfileGains().p(0.1);

    public static final double kTargetAngleLockMinPower = -1.0;
    public static final double kTargetAngleLockMaxPower = 1.0;
    public static final double kTargetAngleLockMinInput = -27;
    public static final double kTargetAngleLockMaxInput = 27;
    public static final double kTargetAngleLockFrictionFeedforward = 0.44 / 12;

}
