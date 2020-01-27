package us.ilite.common.config;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.NetworkTablesConstantsBase;
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

    public static class PowerCellModule {
        public static double kIntakeTalonPower = 1d;
        public static double kForStopTalon = 0d;
        public static int kCANIntakeID = 16; // Change later // Using the bunny bots flywheel id for now
        public static int kTalonOneID; // Change later
        public static int kTalonTwoID; // Change later
        public static int kTalonThreeID; // Change later
        public static int kBeamChannel1; // Change later
        public static int kBeamChannel2; // Change later
        public static int kBeamChannel3; // Change later
        public static double kIntakeCANPower = 1.0;
        public static double kForStopCAN = 1.0;

    }


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

    // =============================================================================
    // Target Constants
    // Note: These constants need to be recalculted for the specific target geometry
    // =============================================================================
    // TODO These values are specific to the targets, not the camera, and may belong elsewhere
    // The current target values assume the limelight processing stream is configured to target
    // the bottom of the vision target
    public enum VisionTarget {
        Generic (10.0), // Used for testing new things
        HatchPort(25.6875), // height of the bottom of the reflective tape in inches for the hatch port
        CargoPort(33.3125), // height of the bottom of the reflective tape in inches for the cargo port
        Ground(0.0), //The ground
        CargoHeight(6.5d);//This may change, not sure what the correct value

        private final double height;

        VisionTarget( double height)  {
            this.height = height;
        }

        /**
         * @return the height
         */
        public double getHeight() {
            return height;
        }
        /**
         * @return the pipelineName
         */
    }

}
