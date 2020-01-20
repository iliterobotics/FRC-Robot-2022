package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.AbstractSystemSettingsUtils;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.PerfTimer;
import us.ilite.common.types.MatchMetadata;
import us.ilite.robot.controller.AbstractController;
import us.ilite.robot.controller.TestController;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.hardware.GetLocalIP;
import us.ilite.robot.modules.FlywheelPrototype;
import us.ilite.robot.modules.ModuleList;
import us.ilite.robot.modules.OperatorInput;
import us.ilite.robot.modules.PowerCellModule;

import static us.ilite.common.types.EMatchMode.*;

import java.util.List;

public class Robot extends TimedRobot {

    private ILog mLogger = Logger.createLog(this.getClass());

    private ModuleList mRunningModules = new ModuleList();
    private Clock mClock = new Clock();
    public static final Data DATA = new Data();
    private Timer initTimer = new Timer();
    private final Settings mSettings = new Settings();
    private PowerCellModule mIntake = new PowerCellModule(DATA);
    private CSVLogger mCSVLogger = new CSVLogger(DATA);

    private PowerDistributionPanel pdp = new PowerDistributionPanel(Settings.Hardware.CAN.kPDP);

    private FlywheelPrototype mFlywheel;
    private OperatorInput mOI;

    private MatchMetadata mMatchMeta = null;

    private PerfTimer mClockUpdateTimer = new PerfTimer();

    private final TestController mTestController = new TestController();
    private AbstractController mActiveController = null;


    @Override
    public void robotInit() {
        mFlywheel = new FlywheelPrototype();
        mIntake = new PowerCellModule(DATA);
        mOI = new OperatorInput();

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
        } catch(Exception e) {
            mLogger.exception(e);
        }

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
    }

    @Override
    public void autonomousPeriodic() {
        commonPeriodic();
    }

    @Override
    public void teleopInit() {
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
        mRunningModules.addModule(mIntake);
        mRunningModules.modeInit(TEST, mClock.getCurrentTime());
        mRunningModules.readInputs(mClock.getCurrentTime());
        mRunningModules.checkModule(mClock.getCurrentTime());
    }

    @Override
    public void testPeriodic() {
        commonPeriodic();
    }

    private void commonPeriodic() {
        double start = Timer.getFPGATimestamp();
        for(Codex c : DATA.mAllCodexes) {
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
        }
    }
}
