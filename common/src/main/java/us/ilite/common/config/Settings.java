package us.ilite.common.config;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import us.ilite.common.lib.control.PIDGains;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.NetworkTablesConstantsBase;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EPowerDistPanel;

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

    public static int sCODEX_COMMS_PORT = 5805;

    // ================================
    // System ID's
    // ================================
    public static class Hardware {
        // =============================================================================
        // IMU Constants
        // =============================================================================


        public static class CAN {
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
        }

        public static class PCM {
            public static int kFourBarDoubleSolenoidForward = 0;
            public static int kFourBarDoubleSolenoidReverse = 1;
            public static int kFourBarPusher = 0;
        }

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

    public static List<ELogitech310> kTeleopCommandTriggers = Arrays.asList(InputMap.DRIVER.TRACK_TARGET_BTN,
                                                                            InputMap.DRIVER.TRACK_CARGO_BTN,
                                                                            InputMap.DRIVER.TRACK_HATCH_BTN);

    public static List<ELogitech310> kAutonOverrideTriggers = Arrays.asList(InputMap.DRIVER.THROTTLE_AXIS,
                                                                            InputMap.DRIVER.TURN_AXIS);
    public static double kAutonOverrideAxisThreshold = 0.3;


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
