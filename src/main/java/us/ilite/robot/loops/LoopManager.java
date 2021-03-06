package us.ilite.robot.loops;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.Clock;

/**
 * A class which uses the WPILIB Notifier mechanic to run our Modules on
 * a set time.  Tune loop period to the desired,
 * but monitor CPU usage.
 */
@Deprecated

public class LoopManager implements Runnable{
    private ILog mLog = Logger.createLog(LoopManager.class);

    private final double kLoopPeriodSeconds;

    private final Notifier mWpiNotifier;
    private final Clock mClock;

    private final LoopList mLoopList = new LoopList();

    private final Object mTaskLock = new Object();
    private boolean mIsRunning = false;
    private long numLoops = 0;
    private long numOverruns = 0;

    public LoopManager(double pLoopPeriodSeconds) {
        mWpiNotifier = new Notifier(this);
        mClock = new Clock();
        this.kLoopPeriodSeconds = pLoopPeriodSeconds;
    }

    public void setRunningLoops(Loop ... pLoops) {
        mLoopList.setLoops(pLoops);
    }

    public synchronized void start() {
        mClock.update();
        if(!mIsRunning) {
            mLog.info("Starting us.ilite.common.lib.control loop");
            synchronized(mTaskLock) {
                mLoopList.modeInit(EMatchMode.TELEOPERATED);
                mLoopList.readInputs();
                mIsRunning = true;
            }
            mWpiNotifier.startPeriodic(kLoopPeriodSeconds);
        }
    }

    public synchronized void stop() {
        
        if(mIsRunning) {
            mLog.info("Stopping us.ilite.common.lib.control loop");
            mWpiNotifier.stop();
            synchronized(mTaskLock) {
                mIsRunning = false;
                mLoopList.shutdown();
            }

            if(numLoops != 0) {
                mLog.error("Experienced ", numOverruns, "/", numLoops, " timing overruns, or ", ((double)numOverruns/(double)numLoops) * 100.0, "%.");
            }
        }

    }

    @Override
    public void run() {
        if(mIsRunning) {
//            loopTimer.reset();
//            loopTimer.start();
            double start = Timer.getFPGATimestamp();
            synchronized (mTaskLock) {

                try {
                    if (mIsRunning) {
//                        inputTimer.reset();
//                        inputTimer.start();
                        mLoopList.readInputs();
//                        inputTimer.stop();
//                        updateTimer.reset();
//                        updateTimer.start();
                        mLoopList.loop(Timer.getFPGATimestamp());
//                        updateTimer.stop();
                    }
                } catch (Throwable t) {
                    t.printStackTrace();
                }
            }
//            mClock.cycleEnded();

//            Data.kSmartDashboard.getEntry("loop_input_dt").setDouble(inputTimer.get());
//            Data.kSmartDashboard.getEntry("loop_update_dt").setDouble(updateTimer.get());

//            loopTimer.stop();
            double dt = Timer.getFPGATimestamp() - start;
            numLoops++;
//            SmartDashboard.putNumber("highfreq_loop_dt", dt);
            if (dt > Settings.kControlLoopPeriod) {
//                mLog.error("Overrun: ", /*loopTimer.get()*/dt, " Input took: "/*, inputTimer.get()*/, " Update took: "/*,updateTimer.get()*/);
                numOverruns++;
            }
        }
    }
    
}
