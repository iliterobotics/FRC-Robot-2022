package us.ilite.robot;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.Field2022;
import us.ilite.common.config.AbstractSystemSettingsUtils;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.MatchMetadata;
import us.ilite.robot.auto.AutonSelection;
import us.ilite.robot.controller.*;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.modules.*;
import us.ilite.robot.network.EForwardableConnections;

import java.util.Arrays;

import static us.ilite.common.types.EMatchMode.*;

public class Robot extends TimedRobot {

    private ILog mLogger = Logger.createLog(this.getClass());
    public static final Data DATA = new Data();
    public static final Clock CLOCK = (RobotBase.isReal() ? new Clock() : new Clock().simulated());
    public static final Field2d FIELD = new Field2d();
    public static final boolean IS_SIMULATED = RobotBase.isSimulation();
    private static EMatchMode MODE = DISABLED;
    private ModuleList mRunningModules = new ModuleList();
    private final Settings mSettings = new Settings();
    private CSVLogger mCSVLogger;
   // private HangerModule mHanger;
    private Timer initTimer = new Timer();

    private DriveModule mDrive;
    private Limelight mLimelight;
    private LEDControl mLEDControl;
    private SimulationModule mSimulation;
    private FeederModule mFeeder;
    private VioletDriveModule mViolet;
    private IntakeModule mIntake;
    private ClimberModule mClimber;

//    private PowerDistributionPanel pdp = new PowerDistributionPanel(Settings.Hardware.CAN.kPDP);

    private OperatorInput mOI;
    private MatchMetadata mMatchMeta = null;

    private final AbstractController mTeleopController = TeleopController.getInstance();
    private AbstractController mBaseAutonController;
    public AutonSelection mAutonSelection;
    private AbstractController mActiveController = null;
    private TestController mTestController;


    @Override
    public void robotInit() {
        CLOCK.update();
        Arrays.stream(EForwardableConnections.values()).forEach(EForwardableConnections::addPortForwarding);

        // Init the actual robot
//        initTimer.reset();
//        initTimer.start();
        mCSVLogger = new CSVLogger( Settings.kIsLogging );
        mBaseAutonController = new BaseAutonController();
     //   mViolet = new VioletDriveModule();
        mDrive = new DriveModule();
        MODE=INITIALIZING;
        mLogger.warn("===> ROBOT INIT Starting");
        mAutonSelection = new AutonSelection();
        mOI = new OperatorInput();
        mLimelight = new Limelight(Settings.kFlywheelLimelightNetworkTable);
        mFeeder = new FeederModule();
        mIntake = new IntakeModule();
        mLEDControl = new LEDControl();
        mClimber = new ClimberModule();
//        mCompressor = new Compressor(20, PneumaticsModuleType.REVPH);
//        mCompressor.enableAnalog(55, 60);
        if(IS_SIMULATED) {
            mSimulation = new SimulationModule();
        }

        //look for practice robot config:
        AbstractSystemSettingsUtils.loadPracticeSettings(mSettings);

        Logger.setLevel(ELevel.WARN);
        mLogger.info("Starting Robot Initialization...");

//        mSettings.writeToNetworkTables();

//        new Thread(new DSConnectInitThread()).start();
        // Init static variables and get singleton instances first

        ICodexTimeProvider provider = new ICodexTimeProvider() {
            @Override
            public double getTimestamp() {
                return CLOCK.now();
            }
        };
        CodexMetadata.overrideTimeProvider(provider);

        mRunningModules.clearModules();

        LiveWindow.disableAllTelemetry();

        initTimer.stop();
        mLogger.warn("Robot initialization finished. Took: ", initTimer.get(), " seconds");

        if ( !Settings.kIsLogging ) {
            mLogger.warn("------------Not Logging to CSV------------");
        }

    }

    /**
     * This contains code run in ALL robot modes.
     * It's also important to note that this runs AFTER mode-specific code
     */
    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        if ( Settings.kIsLogging ) {
            mCSVLogger.start();
        }

        MODE=AUTONOMOUS;
        mActiveController = new DriveStraightTurnController();
        mActiveController.setEnabled(true);
        mRunningModules.clearModules();
//        mRunningModules.addModule(mLimelight);
      //  mRunningModules.addModule(mFeeder);
      //  mRunningModules.addModule(mIntake);
        mRunningModules.addModule(mDrive);
        mRunningModules.modeInit(AUTONOMOUS);
    }

    @Override
    public void autonomousPeriodic() {
        commonPeriodic();
    }

    @Override
    public void teleopInit() {
        if ( Settings.kIsLogging ){
            mCSVLogger.start();
        }

        mRunningModules.clearModules();
        mRunningModules.addModule(mOI);
        mRunningModules.addModule(mFeeder);
//        mRunningModules.addModule(mViolet);
        mRunningModules.addModule(mIntake);
        mRunningModules.addModule(mDrive);
//        mRunningModules.addModule(mHanger);
//        mRunningModules.addModule(mLimelight);
//        mRunningModules.addModule(mClimber);
        MODE=TELEOPERATED;
        mActiveController = mTeleopController;
        mActiveController.setEnabled(true);
        mRunningModules.modeInit(TELEOPERATED);
    }

    @Override
    public void teleopPeriodic() {
        commonPeriodic();
//        System.out.println(mCompressor.enabled());
    }

    @Override
    public void disabledInit() {
        MODE=DISABLED;
        mLogger.info("Disabled Initialization");

        mRunningModules.shutdown();
        // Don't clear the modules - we want them to continue updating shuffleboard with sensor readings

//        mCSVLogger.stop();

        if(mActiveController != null) {
            mActiveController.setEnabled(false);
        }
    }

    @Override
    public void disabledPeriodic() {
        mOI.readInputs();
//        mDrive.readInputs();
        mIntake.readInputs();
        mFeeder.readInputs();
        Shuffleboard.update();
    }

    @Override
    public void testInit() {
        if ( Settings.kIsLogging ){
            mCSVLogger.start();
        }

        if(mTestController == null) {
            mTestController = TestController.getInstance();
        }
        MODE = TEST;
        mActiveController = mTestController;
        mActiveController.setEnabled(true);

        mRunningModules.clearModules();
        mRunningModules.addModule(mOI);
//        mRunningModules.addModule(mLimelight);
//        mRunningModules.addModule(mFeeder);
//        mRunningModules.addModule(mDrive);
//        mRunningModules.addModule(mHanger);
//        mRunningModules.addModule(mIntake);
        mRunningModules.addModule(mLEDControl);
        if(IS_SIMULATED) {
            mRunningModules.addModule(mSimulation);
        }
//        mRunningModules.addModule(mViolet);
//        mRunningModules.addModule(mHanger);
//        mRunningModules.addModule(mIntake);
//        mRunningModules.addModule(mLEDControl);
//        if(IS_SIMULATED) {
//            mRunningModules.addModule(mSimulation);
//        }
        mRunningModules.modeInit(TEST);
        mRunningModules.checkModule();
    }

    @Override
    public void testPeriodic() {
        //Used to Test logging
//        for (RobotCodex c : DATA.mLoggedCodexes ) {
//            if (c.equals(DATA.drivetrain)) {
//                DATA.randomizeCodex(c);
//            }
//        }
        commonPeriodic();
    }

    void commonPeriodic() {
        double start = Timer.getFPGATimestamp();
        CLOCK.update();
        if ( Settings.kIsLogging && MODE != DISABLED) {
            for ( RobotCodex c : DATA.mLoggedCodexes ) {
                if ( c.hasChanged() ) {
                    mCSVLogger.addToQueue( new Log( c.toFormattedCSV(), c.meta().gid()) );
                }
            }
        }
        for ( RobotCodex c : DATA.mAllCodexes ) {
            c.reset();
        }
//        EPowerDistPanel.map(mData.pdp, pdp);

        mRunningModules.readInputs();
        mActiveController.update();
        mRunningModules.setOutputs();
        SmartDashboard.putNumber("common_periodic_dt", Timer.getFPGATimestamp() - start);
        SmartDashboard.putNumber("Clock Time", CLOCK.now());
    }

    private void initMatchMetadata() {
        if (mMatchMeta == null) {
            mMatchMeta = new MatchMetadata();
            int gid = mMatchMeta.hash;
            for (RobotCodex c : DATA.mAllCodexes) {
                c.meta().setGlobalId(gid);
            }
        }
    }

    public static EMatchMode mode() {
        return MODE;
    }

    public String toString() {

        String mRobotMode = "Unknown";
        String mRobotEnabledDisabled = "Unknown";
        double mNow = Timer.getFPGATimestamp();

        if (this.isAutonomous()) {
            mRobotMode = "Autonomous";
        }
//        if (this.isOperatorControl()) {
//            mRobotMode = "OPERATOR Control";
//        }
        if (this.isTest()) {
            mRobotEnabledDisabled = "Test";
        }
        if (this.isEnabled()) {
            mRobotEnabledDisabled = "Enabled";
        }
        if (this.isDisabled()) {
            mRobotEnabledDisabled = "Disabled";
        }

        return String.format("State: %s\tMode: %s\tTime: %s", mRobotEnabledDisabled, mRobotMode, mNow);

    }


}
