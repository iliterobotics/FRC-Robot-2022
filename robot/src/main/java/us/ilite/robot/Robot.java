package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.AbstractSystemSettingsUtils;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.PerfTimer;
import us.ilite.common.types.MatchMetadata;
import us.ilite.robot.commands.DJBoothPositionControl;
import us.ilite.robot.commands.DJBoothRotationControl;
import us.ilite.robot.controller.AbstractController;
import us.ilite.robot.controller.BaseAutonController;
import us.ilite.robot.controller.TeleopController;
import us.ilite.robot.controller.TestController;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.GetLocalIP;
import us.ilite.robot.modules.*;

import static us.ilite.common.types.EMatchMode.*;
import java.util.List;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

    private ILog mLogger = Logger.createLog(this.getClass());

    private ModuleList mRunningModules = new ModuleList();
    private Clock mClock = new Clock();
    public static final Data DATA = new Data();
    private Timer initTimer = new Timer();
    private final Settings mSettings = new Settings();
    private CSVLogger mCSVLogger = new CSVLogger(DATA);

    private PowerDistributionPanel pdp = new PowerDistributionPanel(Settings.Hardware.CAN.kPDP);

    private DriveModule mDrive;
    private FlywheelPrototype mFlywheel;
    private OperatorInput mOI;
    private Limelight mLimelight;
    private MatchMetadata mMatchMeta = null;

    private PerfTimer mClockUpdateTimer = new PerfTimer();

    private final AbstractController mTestController = new TestController();
    private final AbstractController mTeleopController = new TeleopController();
    private final AbstractController mBaseAutonController = new BaseAutonController();
    private AbstractController mActiveController = null;


    private ColorSensorV3 mColorSensorV3;
    private final ColorMatch m_colorMatcher = new ColorMatch();

    // Ideally, these values would be put into SystemSettings, however, that would require vendordeps
    // and a gradle build for that vendordeps file
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


    @Override
    public void robotInit() {
        mFlywheel = new FlywheelPrototype();
        mOI = new OperatorInput( new DJBoothRotationControl(), new DJBoothPositionControl());
        mDrive = new DriveModule();
        mLimelight = new Limelight(DATA);

        //look for practice robot config:
        AbstractSystemSettingsUtils.loadPracticeSettings(mSettings);

        // Init the actual robot
        initTimer.reset();
        initTimer.start();
        Logger.setLevel(ELevel.WARN);
        mLogger.info("Starting Robot Initialization...");

        mSettings.writeToNetworkTables();

//        new Thread(new DSConnectInitThread()).start();
        // Init static variables and get singleton instances first

        ICodexTimeProvider provider = new ICodexTimeProvider() {
            public long getTimestamp() {
                return (long) mClock.getCurrentTimeInNanos();
            }
        };
        CodexMetadata.overrideTimeProvider(provider);

        mRunningModules.clearModules();

        try {
        } catch (Exception e) {
            mLogger.exception(e);
        }

        LiveWindow.disableAllTelemetry();

        //Getting Color Sensor
        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);

        //Setting Color Matcher with Target Colors
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);

        initTimer.stop();
        mLogger.info("Robot initialization finished. Took: ", initTimer.get(), " seconds");
    }

    /**
     * This contains code run in ALL robot modes.
     * It's also important to note that this runs AFTER mode-specific code
     */
    @Override
    public void robotPeriodic() {
        mClock.cycleEnded();

        Color detectedColor = mColorSensorV3.getColor();

        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        SmartDashboard.putString( "Detected Color: ", colorString );
    }

    @Override
    public void autonomousInit() {
        mActiveController = mBaseAutonController;
    }

    @Override
    public void autonomousPeriodic() {
        commonPeriodic();
    }

    @Override
    public void teleopInit() {
        mActiveController = mTeleopController;
    }

    @Override
    public void teleopPeriodic() {
        commonPeriodic();
    }

    @Override
    public void disabledInit() {
        mLogger.info("Disabled Initialization");
        mRunningModules.shutdown(mClock.getCurrentTime());
        mCSVLogger.stop(); // stop csv logging
        mActiveController = null;
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
        mActiveController = mTestController;
        mRunningModules.clearModules();
        mRunningModules.addModule(mOI);
        mRunningModules.addModule(mFlywheel);
        mRunningModules.addModule(mDrive);
        mRunningModules.modeInit(TEST, mClock.getCurrentTime());
        mRunningModules.readInputs(mClock.getCurrentTime());
        mRunningModules.checkModule(mClock.getCurrentTime());
    }

    @Override
    public void testPeriodic() {
        commonPeriodic();
    }

    void commonPeriodic() {
        double start = Timer.getFPGATimestamp();
        for (Codex c : DATA.mAllCodexes) {
            c.reset();
        }
//        EPowerDistPanel.map(mData.pdp, pdp);
        mRunningModules.readInputs(mClock.getCurrentTime());
        mActiveController.update(mClock.getCurrentTime());
        mRunningModules.setOutputs(mClock.getCurrentTime());
//        mData.sendCodicesToNetworkTables();
        SmartDashboard.putNumber("common_periodic_dt", Timer.getFPGATimestamp() - start);
    }

    private void initMatchMetadata() {
        if (mMatchMeta == null) {
            mMatchMeta = new MatchMetadata();
            int gid = mMatchMeta.hash;
            for (Codex c : DATA.mAllCodexes) {
                c.meta().setGlobalId(gid);
            }
        }
    }

    public String toString() {

        String mRobotMode = "Unknown";
        String mRobotEnabledDisabled = "Unknown";
        double mNow = Timer.getFPGATimestamp();

        if (this.isAutonomous()) {
            mRobotMode = "Autonomous";
        }
        if (this.isOperatorControl()) {
            mRobotMode = "OPERATOR Control";
        }
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

    public Color getColor() {
        Color detectedColor = mColorSensorV3.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        return match.color;
    }
}

