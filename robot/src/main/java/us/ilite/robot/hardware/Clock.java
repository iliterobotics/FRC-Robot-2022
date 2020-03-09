package us.ilite.robot.hardware;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Consumer;

/**
 * @author Stephen Welch
 * Provides a consistent time between cycles.
 * The cycleEnded() method must be called at the end of each robot cycle so we know when to update to the next time.
 */
public class Clock {

    private ILog mLogger = Logger.createLog(Clock.class);

    private double mStartTime = 0.0;
    private double mCurrentTime = 0.0;
    private double mDT = 0.0;
    private boolean mIsSimulated = false;

    public Clock() {
    }

    public void update() {
        double t = (mIsSimulated) ? getJavaTime() : getRobotTime() - mStartTime;
        mDT = t - mCurrentTime;
        mCurrentTime = t;
    }

    /**
     *
     * @return A cycle-consistent time, in seconds.
     */
    public double now() {
        return mCurrentTime;
    }


    /**
     *
     * @return A cycle-consistent delta-time, in seconds.
     */
    public double dt() {
        return mDT;
    }

    /**
     *
     * @return A cycle-consistent time, in milliseconds.
     */
    public double getCurrentTimeInMillis() {
        return now() * 1e3;
    }

    /**
     *
     * @return A cycle-consistent time, in microseconds.
     */
    public double getCurrentTimeInMicros() {
        return now() * 1e6;
    }

    /**
     *
     * @return A cycle-consistent time, in nanoseconds.
     */
    public double getCurrentTimeInNanos() {
        return now() * 1e9;
    }

    public void setTime(double time) {
        if(mIsSimulated) {
            mStartTime = 0.0;
            mCurrentTime = time;
        } else {
            mLogger.error("Setting the current time is not allowed outside of simulation.");
        }
    }

    public Clock simulated() {
        mStartTime = getJavaTime();
        mIsSimulated = true;
        return this;
    }

    private static double getJavaTime() {
        return System.currentTimeMillis() / 1000.0;
    }

    private static double getRobotTime() {
        return Timer.getFPGATimestamp();
    }

    public void report(String pName, Consumer<Void> pTodo) {
        double start = Timer.getFPGATimestamp();
        pTodo.accept(null);
        double end = Timer.getFPGATimestamp();
        SmartDashboard.putNumber(pName, end-start);
    }

}
