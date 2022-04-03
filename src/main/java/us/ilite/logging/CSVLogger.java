package us.ilite.logging;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.Settings;
import us.ilite.common.types.MatchMetadata;
import us.ilite.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

/**
 * Class responsible for managing all of the {@link CSVWriter}s. Logging events
 * are sent to this class in the form of ImmutablePair of the String to log,
 * which is expected to be CSV format, and the {@link RobotCodex} in the CSVLogger#logFromCodexToCSVLog(ImmutablePair)
 *
 * This class will run a timer thread that will periodically look for log events and will pass them onto
 * the specific {@link CSVWriter} for the event's {@link RobotCodex}
 */
public class CSVLogger {
    public static LinkedBlockingDeque<Log> kCSVLoggerQueue = new LinkedBlockingDeque<>();
    private List<CSVWriter> mCSVWriters;
    private ILog mLogger = Logger.createLog(this.getClass());
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "My timer thread"));
    private ScheduledFuture<?> scheduledFuture;
    private boolean mIsAcceptingToQueue;

    public CSVLogger(boolean pIsLogging, MatchMetadata pMetadata) {
        if ( pIsLogging ) {
            mCSVWriters = new ArrayList<>();
            for (RobotCodex c : Robot.DATA.mLoggedCodexes) {
                mCSVWriters.add(new CSVWriter(c));
            }
            mIsAcceptingToQueue = false;
            logFromCodexToCSVHeader();
            scheduledFuture = mExService.scheduleAtFixedRate(this::run, Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
        }
        else {
            mIsAcceptingToQueue = false;
        }
    }

    private void run() {
        if ( !kCSVLoggerQueue.isEmpty() ) {
            try {
                ArrayList<Log> kTempCSVLogs = new ArrayList<>();
                kCSVLoggerQueue.drainTo(kTempCSVLogs);

                for ( Log log : kTempCSVLogs ) {
                    //TODO - fix the excessive exceptions
                    logFromCodexToCSVLog( log );
                }
            } catch (Exception e) {}
        }
    }

    public void logFromCodexToCSVHeader() {
        mCSVWriters.forEach(c -> c.writeHeader());
    }

    public void logFromCodexToCSVLog( Log pLog ) {
        for ( CSVWriter c : mCSVWriters ) {
            if ( c.getMetaDataOfAssociatedCodex().gid() == pLog.getmGlobalId() ) {
                c.log( pLog.getmLogData() );
                break;
            }
        }
    }

    /**
     * Opens the queue
     */
    public void start() {
        mIsAcceptingToQueue = true;
    }

    /**
     * Closes the queue
     */
    public void stop() {
        mIsAcceptingToQueue = false;
    }

    public void addToQueue( Log pLog ) {
        if(mIsAcceptingToQueue) {
            kCSVLoggerQueue.add( pLog );
        } else {
        }
    }

}