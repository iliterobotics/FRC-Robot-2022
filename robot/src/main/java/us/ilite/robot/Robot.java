package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.AbstractSystemSettingsUtils;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.PerfTimer;
import us.ilite.common.types.MatchMetadata;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.GetLocalIP;
import us.ilite.robot.hardware.VisionGyro;
import us.ilite.robot.loops.LoopManager;
import us.ilite.robot.modules.CommandManager;
import us.ilite.robot.modules.Drive;
import us.ilite.robot.modules.Limelight;
import us.ilite.robot.modules.ModuleList;

import java.util.List;

public class Robot extends TimedRobot {

    private ILog mLogger = Logger.createLog(this.getClass());

    private LoopManager mLoopManager = new LoopManager(Settings.kControlLoopPeriod);
    // It sure would be convenient if we could reduce this to just a LoopManager...Will have to test timing of Codex first
    private LoopManager mLoopManagerx = new LoopManager(Settings.kControlLoopPeriod);
    private ModuleList mRunningModules = new ModuleList();

    private Clock mClock = new Clock();
    private Data mData = new Data();
    private Timer initTimer = new Timer();
    private final Settings mSettings = new Settings();
    private CSVLogger mCSVLogger = new CSVLogger(mData);

    private PowerDistributionPanel pdp = new PowerDistributionPanel(Settings.Hardware.CAN.kPDP);


    // Module declarations here
    private CommandManager mAutonomousCommandManager = new CommandManager().setManagerTag("Autonomous Manager");
    private CommandManager mTeleopCommandManager = new CommandManager().setManagerTag("Teleop Manager");

    private Drive mDrive = new Drive(mData);
    private Limelight mLimelight = new Limelight(mData);
    private VisionGyro mVisionGyro = new VisionGyro(mData);


    private MatchMetadata mMatchMeta = null;

    private PerfTimer mClockUpdateTimer = new PerfTimer();

    @Override
    public void robotInit() {
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

        mRunningModules.setModules();

        try {
        } catch(Exception e) {
            mLogger.exception(e);
        }

        mData.registerCodices();
        LiveWindow.disableAllTelemetry();

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
    }

    @Override
    public void autonomousInit() {
        initMatchMetadata(); // TODO - move this to a DS connection thread
        initTimer.reset();
        initTimer.start();
        mLogger.info("Starting Autonomous Initialization...");

        mSettings.loadFromNetworkTables();

        // Init modules after commands are set
        mRunningModules.modeInit(mClock.getCurrentTime());
        mRunningModules.periodicInput(mClock.getCurrentTime());

        mLoopManager.setRunningLoops(mLimelight, mDrive);
        mLoopManager.start();

//        mAutonomousCommandManager.startCommands(mAutonomousRoutines.getDefault());

        mData.registerCodices();
//        mCSVLogger.start(); // Start csv logging

        initTimer.stop();
        mLogger.info("Autonomous initialization finished. Took: ", initTimer.get(), " seconds");
    }

    @Override
    public void autonomousPeriodic() {
        commonPeriodic();
    }

    @Override
    public void teleopInit() {
        initMatchMetadata();

        mSettings.loadFromNetworkTables();

        mRunningModules.modeInit(mClock.getCurrentTime());
        mRunningModules.periodicInput(mClock.getCurrentTime());

        mLoopManager.setRunningLoops(mLimelight, mDrive);
        mLoopManager.start();

//        mCSVLogger.start(); // start csv logging
    }

    @Override
    public void teleopPeriodic() {
        commonPeriodic();
//        mData.sendCodices();
    }

    @Override
    public void disabledInit() {
        mLogger.info("Disabled Initialization");
        mRunningModules.shutdown(mClock.getCurrentTime());
        mLoopManager.stop();
        mCSVLogger.stop(); // stop csv logging
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
        mRunningModules.setModules(mDrive);
        mRunningModules.modeInit(mClock.getCurrentTime());
        mRunningModules.periodicInput(mClock.getCurrentTime());
        mRunningModules.checkModule(mClock.getCurrentTime());

        mLoopManager.start();
    }

    @Override
    public void testPeriodic() {


    }

    private void commonPeriodic() {
        double start = Timer.getFPGATimestamp();
        for(Codex c : mData.mAllCodexes) {
            c.reset();
        }
        EPowerDistPanel.map(mData.pdp, pdp);
        mRunningModules.periodicInput(mClock.getCurrentTime());
        mRunningModules.update(mClock.getCurrentTime());
//        mData.sendCodicesToNetworkTables();
        SmartDashboard.putNumber("common_periodic_dt", Timer.getFPGATimestamp() - start);
    }

    private void initMatchMetadata() {
        if (mMatchMeta == null) {
            mMatchMeta = new MatchMetadata();
            int gid = mMatchMeta.hash;
            for (Codex c : mData.mAllCodexes) {
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

    private class DSConnectInitThread implements Runnable {

        @Override
        public void run() {

            while(!DriverStation.getInstance().isDSAttached()) {
                try {
                    mLogger.error("Waiting on Robot <--> DS Connection...");
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            List<String> ips = GetLocalIP.getAllIps();
            mData.initCodexSender(ips);
        }
    }
}
