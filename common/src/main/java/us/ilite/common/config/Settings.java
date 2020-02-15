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

    public static double kControlLoopPeriod = 0.01; // seconds

    public static int kSecondsToUpdateCSVLogger = 1; //seconds

    public static double kNetworkTableUpdateRate = 0.01;

    public static double kDJOutput = .25;

    public static int sCODEX_COMMS_PORT = 5805;

    public static final String AUTO_PATH_PACKAGE = "us.ilite.robot.auto.paths";

    // ================================
    // System ID's
    // ================================
    public static class Hardware {
        // =============================================================================
        // IMU Constants
        // =============================================================================


        public static class CAN {
            public static int kDJBoothTalonId = 72;
            public static int kHangerNeoID1 = 15;
            public static int kHangerNeoID2 = 71;
//            public static int kTalonThreeID = 65; // Change later
            public static final int kTurretGyroID = 61; // There isn't a gyro on the BunnyBot
            public static final int kShooterID = 66; // BunnyBot Shooter
            public static final int kAcceleratorID = 61; // BunnyBot Conveyor
            public static final int kTurretID = 69; // BunnyBot Hopper
            public static final int kDJSpinnerVictorID = 50;
            public static int kTimeoutMs = 10; //use for on the fly updates
            public static int kLongTimeoutMs = 100; //use for constructors

//            public static double kGyroCollisionThreshold = 0.0;

            // =============================================
            // DO NOT CHANGE ANY OF THE FOLLOWING CAN ID's
            // =============================================
            public static int kPDP = 20;
            public static int kPigeon = 21;
            // ===== 2020 Drive =====
            public static int kDriveRightMaster = 1;
            public static int kDriveRightFollower = 2;
            public static int kDriveLeftMaster = 3;
            public static int kDriveLeftFollower = 4;

            public static int kMAXIntakeRollerId = 7;
            public static int kMAXIntakeArm = 8;
            public static int kTalonPowerCellSerializer = 11;
            public static int kTalonVerticalID = 12;
            public static int kLEDControlCanifierID = 0;

            // ===== 2019 Drive =====
//            public static  int kDriveLeftMaster = 1;
//            public static int kDriveLeftFollower = 3;
//            public static  int kDriveRightMaster = 2;
//            public static int kDriveRightFollower = 4;

        }

        public static class Analog {
        }

        public static class DIO {
            public static int kEntryBeamChannel = 9;
            public static int kSecondaryBeamChannel = 8; // Change later
            public static int kExitBeamChannel = 7; // Change later
            public static final int kAnglerID = 8; // BunnyBot Catapult
        }

        public static class PCM {
            public static int kFourBarDoubleSolenoidForward = 0;
            public static int kFourBarDoubleSolenoidReverse = 1;
            public static int kFourBarPusher = 0;
        }

    }

    public static class Input {

        public static double kNormalPercentThrottleReduction = 1.0;
        // These are applied AFTER the normal throttle reduction
        public static double kSnailModePercentThrottleReduction = 0.5;
        public static double kSnailModePercentRotateReduction = 0.4;
        public static double kMaxAllowedVelocityMultiplier = 0.15;

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
