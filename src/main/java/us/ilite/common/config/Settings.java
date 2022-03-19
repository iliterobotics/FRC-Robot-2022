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
    public static final String kFlywheelLimelightNetworkTable = "limelight-top";
    public static double kControlLoopPeriod = 0.02; // seconds
    public static int kSecondsToUpdateCSVLogger = 1; //seconds
    public static double kNetworkTableUpdateRate = 0.01;
    public static boolean kIsLogging = true; // decide whether or not to log
    public static int sCODEX_COMMS_PORT = 5805;

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
            public static final int kDTL3 = 3;
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
            public static final int kLEDCanifier = 31;
        }

        public static class Analog {
        }

        public static class DIO {
            public static final int kEDTLA = 3;
            public static final int kEDTLB = 2;
            public static final int kEDTRA = 1;
            public static final int kEDTRB = 0;
            public static final int kINEntryBeam = 4;
            public static final int kINExitBeam = 10;
        }

        public static class PCH {
            public static final int kPCHCompressorModule = 20;
            // ======================
            // FEEDER AND INTAKE ID's
            // ======================
            public static final int kINPNIntakeForward = 0;
            public static final int kINPNIntakeReverse = 1;

            // ============
            // CLIMBER ID's
            // ============
            public static final int kCLPNClimbForwardOne = 2;
            public static final int kCLPNClimbReverseOne = 3;

            public static final int kCLPNClimbForwardTwo = 4;
            public static final int kCLPNClimbReverseTwo = 5;
        }
    }

    public static class Input {
        public static double kNormalPercentThrottleReduction = 1.0;
        // These are applied AFTER the normal throttle reduction
        public static double kSnailModePercentThrottleReduction = 0.5;
        public static double kSnailModePercentRotateReduction = 0.4;
        public static double kMaxAllowedVelocityMultiplier = 1.0;
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

    // =============================================================================
    // RamseteCommand constants
    // =============================================================================
    public static final double kS = 0.64353;
    public static final double kV = 0.00046589;
    public static final double kA = 3.2696 * Math.pow(10.0, -5.0);

    public static final double kP = 0.1139;

    public static final double kTrackWidthMeters = 0.6858;
    public static final double kMaxSpeedMetersPerSecond = 15;
    public static final double kMaxAccelerationMetersPerSecondSquared = 15;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
