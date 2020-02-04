package us.ilite.common.config;

import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.NetworkTablesConstantsBase;

import java.util.Arrays;
import java.util.List;

/**
 * ONLY ROBOT-wide settings should go into this class (e.g. things like
 *  - CAN/DIO/etc ID's
 *  - IP ports, addresses
 *  - Driver Input
 *  - Field element locations & vision target heights
 */
public class Settings extends NetworkTablesConstantsBase {

    public static double kControlLoopPeriod = 0.01; // seconds
    public static double kCSVLoggingPeriod = 0.02;  // seconds

    public static double kNetworkTableUpdateRate = 0.01;

    public static double kDJOutput = .25;
    public static int kDJBoothTalonId = 12;

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
            public static int kHangerNeoID1;
            public static int kHangerNeoID2;
            public static int kArmNEOAdress = 16;
            public static int kCANIntakeID = 4; // Change later // Using the bunny bots flywheel id for now
            public static int kTalonOneID = 3; // Change later
            public static int kTalonTwoID = 7; // Change later
            public static int kTalonThreeID = 5; // Change later
            public static final int kTurretGyroID = 1; // There isn't a gyro on the BunnyBot
            public static final int kShooterID = 16; // BunnyBot Shooter
            public static final int kAcceleratorID = 11; // BunnyBot Conveyor
            public static final int kAnglerID = 9; // BunnyBot Catapult
            public static final int kTurretID = 9; // BunnyBot Hopper
            public static int kTimeoutMs = 10; //use for on the fly updates
            public static int kLongTimeoutMs = 100; //use for constructors

            public static int kPCM = 20;
            public static int kPDP = 21;
            public static int kPigeon = 30;
//            public static double kGyroCollisionThreshold = 0.0;

            public static  int kDriveLeftMaster = 1;
            public static int kDriveLeftMiddle = 3;
            public static  int kDriveRightMaster = 2;
            public static int kDriveRightMiddle = 4;

        }

        public static class Analog {
        }

        public static class DIO {
            public static int kBeamChannel1 = 1; // Change later
            public static int kBeamChannel2 = 2; // Change later
            public static int kBeamChannel3 = 3; // Change later
        }

        public static class PCM {
            public static int kFourBarDoubleSolenoidForward = 0;
            public static int kFourBarDoubleSolenoidReverse = 1;
            public static int kFourBarPusher = 0;
        }

    }
    public static class Arm {
        // =============================================================================
        // IMU Constants
        // =============================================================================

//        public static int kArmNeoAddress = 16;

    }

    //==============================================================================
    // Logging
    // =============================================================================

    // =============================================================================
    // Drive Train Constants
    // =============================================================================
    public static class Drive {
        public static double kDriveTrainMaxVelocity = 5676;

        // TODO Find out what units this is in
        public static double kMaxHeadingChange = 5;
    }

    public static class Input {

        public static double kNormalPercentThrottleReduction = 1.0;
        // These are applied AFTER the normal throttle reduction
        public static double kSnailModePercentThrottleReduction = 0.5;
        public static double kSnailModePercentRotateReduction = 0.4;

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
