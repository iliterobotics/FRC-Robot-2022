package us.ilite.common.config;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

    public static double kDJOutput = .25;

    public static int sCODEX_COMMS_PORT = 5805;

    public static final String AUTO_PATH_PACKAGE = "us.ilite.robot.auto.paths";

    public static final String CONTROLLER_PATH_PACKAGE = "us.ilite.robot.controller";

    // ================================
    // System ID's
    // DO NOT CHANGE ANY OF THE FOLLOWING VALUES. PERIOD.
    // ================================
    public static class HW {
        public static class CAN {
            public static int kTimeoutMs = 10; //use for on the fly updates
            public static int kLongTimeoutMs = 100; //use for constructors

            public static final int kDriveRightMaster = 1;
            public static final int kDriveRightFollower = 2;
            public static final int kDriveLeftMaster = 3;
            public static final int kDriveLeftFollower = 4;
            // TODO - double check that this is correct when the robot is right-side-up.
            public static final int kMAXHanger1_left = 5;
            public static final int kMAXHanger2_right = 6;
            public static final int kMAXIntakeRollerId = 7;
            public static final int kMAXIntakeArm = 8;
            public static final int kSRXTurretId = 9;
            public static final int kMAXFeederId = 10;
            public static final int kTalonPowerCellSerializer = 11;
            public static final int kTalonVerticalID = 12;
            public static final int kFalconMasterId = 13;
            public static final int kFalconFollowerId = 14;
            public static final int kDJSpinnerVictorID = 15;

            public static final int kPDP = 20;
            public static final int kPigeon = 21;

            public static int kLEDControlCanifier = 40;
        }

        public static class Analog {
        }

        public static class DIO {
            public static final int kEntryBeamChannel = 9;
            public static final int kSecondaryBeamChannel = 8; // Change later
            public static final int kExitBeamChannel = 7; // Change later
            public static final int kAnglerID = 8; // BunnyBot Catapult
        }

        public static class PWM {
            public static final int kHoodServoId = 9;
        }

        public static class PCM {
        }

    }

    public static class Input {

        public static double kNormalPercentThrottleReduction = 1.0;
        // These are applied AFTER the normal throttle reduction
        public static double kSnailModePercentThrottleReduction = 0.5;
        public static double kSnailModePercentRotateReduction = 0.4;
        public static double kMaxAllowedVelocityMultiplier = 0.5;

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
    public static final double kS = 0.33136;
    public static final double kV = 0.1398;
    public static final double kA = 0.0076288;

    public static final double kP = 0.1139;

    public static final double kTrackwidthMeters = 0.5969;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 15;
    public static final double kMaxAccelerationMetersPerSecondSquared = 15;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
