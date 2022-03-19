package us.ilite.logging;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ilite.common.config.Settings;
import us.ilite.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;

/**
 * Class responsible for managing all of the {@link CSVWriter}s. Logging events 
 * are sent to this class in the form of {@link ImmutablePair} of the String to log, 
 * which is expected to be CSV format, and the {@link RobotCodex} in the {@link CSVLogger#logFromCodexToCSVLog(ImmutablePair)}
 *
 * This class will run a timer thread that will periodically look for log events and will pass them onto
 * the specific {@link CSVWriter} for the event's {@link RobotCodex}
 */
public class CSVLogger {
    /**
     * The main event queue for all Logging events. Each event is the CSV, represented as a string and the
     * {@link RobotCodex} to apply it to. The queue is pulled from the timer thread on a regular interval
     */
    private final LinkedBlockingDeque<ImmutablePair<String,RobotCodex>> mLogQueue;
    /**
     * A map where the key is the unique identifier of the {@link RobotCodex} and the value is the
     * {@link CSVWriter} that will write to the specific file of the {@link RobotCodex}
     */
    private final Map<Integer, CSVWriter>mCSVWriters = new ConcurrentHashMap<>();
    /**
     * The logger that's used to log errors; this is the coding logger
     */
    private final ILog mLogger = Logger.createLog(this.getClass());
    /**
     * Timer thread pool that is used to schedule the event queue
     */
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "CSVLogger-EventThread"));
    /**
     * Network table for the CSV logger to log logging state
     */
    private final NetworkTable mCSVNetworkTable;
    /**
     * The future used to monitor the event queue threads
     */
    private ScheduledFuture<?> scheduledFuture;
    /**
     * Flag indicating whether the logging should occur
     */
    private boolean mIsAcceptingToQueue;

    /**
     * Constructs the {@link CSVLogger} and sets the initial logging thread
     * @param pIsLogging
     */
    public CSVLogger( boolean pIsLogging ) {
        mCSVNetworkTable = NetworkTableInstance.getDefault().getTable("CSVLogger");
        LinkedBlockingDeque<ImmutablePair<String,RobotCodex>>queue = null;
        if ( pIsLogging ) {
            queue = new LinkedBlockingDeque<>();
            for (RobotCodex c : Robot.DATA.mLoggedCodexes) {
                mCSVWriters.put(c.meta().gid(),new CSVWriter(this,c));
            }
            mIsAcceptingToQueue = false;
            logFromCodexToCSVHeader();
            scheduledFuture = mExService.scheduleAtFixedRate(this::run, Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
        }

        mLogQueue = queue;
    }

    /**
     * Event queue thread method
     */
    private void run() {
        if ( !mLogQueue.isEmpty() ) {
            try {
                List<ImmutablePair<String,RobotCodex>> kTempCSVLogs = new ArrayList<>();
                mLogQueue.drainTo(kTempCSVLogs);
                for ( ImmutablePair<String,RobotCodex> log : kTempCSVLogs ) {
                    logFromCodexToCSVLog( log );
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Method to write all of the headers
     */
    private void logFromCodexToCSVHeader() {
        for(CSVWriter writer : mCSVWriters.values()) {
            writer.logCSVLine(writer.getCodex().getCSVHeader());
        }
    }

    /**
     * Helper method to log an event to a specific {@link CSVWriter}
     * @param pLogEvent
     *  The log event
     */
    private void logFromCodexToCSVLog( ImmutablePair<String,RobotCodex> pLogEvent ) {
        String type = ""+pLogEvent.getRight().meta().getEnum();
        CSVWriter writer = mCSVWriters.get(pLogEvent.getRight().meta().gid());
        if(writer != null) {
            mCSVNetworkTable.getEntry("logFromCodexToCSVLog-"+type).setBoolean(true);
            writer.logCSVLine(pLogEvent.getLeft());
        } else {
            mCSVNetworkTable.getEntry("logFromCodexToCSVLog-"+type).setBoolean(false);
        }
    }
    public void start() {
        mIsAcceptingToQueue = true;
    }
    public void addToQueue( ImmutablePair<String,RobotCodex> pLog ) {
        boolean hasQueue = false;
        if(mLogQueue != null && mIsAcceptingToQueue) {
            mLogQueue.add(pLog);
            hasQueue = true;
        }
//        SmartDashboard.putBoolean("LoggerHasQueue-"+pLog.getRight().meta().getEnum(),hasQueue);
        mCSVNetworkTable.getEntry("LoggerHasQueue-"+pLog.getRight().meta().getEnum()).setBoolean(hasQueue);
    }
}