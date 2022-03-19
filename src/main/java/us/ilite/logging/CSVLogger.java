package us.ilite.logging;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.Settings;
import us.ilite.logging.CSVWriter;
import us.ilite.logging.Log;
import us.ilite.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

public class CSVLogger {
    public static LinkedBlockingDeque<Log> kCSVLoggerQueue = new LinkedBlockingDeque<>();
    private List<CSVWriter> mCSVWriters;
    private ILog mLogger = Logger.createLog(this.getClass());
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "My timer thread"));
    private ScheduledFuture<?> scheduledFuture;
    private boolean mIsAcceptingToQueue;

    public CSVLogger( boolean pIsLogging ) {
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
                //mLogger.error( "Drained queue got: " + kTempCSVLogs.size() );

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
        boolean foundLog =false;
        for ( CSVWriter c : mCSVWriters ) {
            if ( c.getMetaDataOfAssociatedCodex().gid() == pLog.getmGlobalId() ) {
                foundLog = true;
                c.log( pLog.getmLogData() );
                break;
            }
        }

        if(!foundLog) {
            System.err.println("ERROR: FAILED TO FIND A LOG TO WRITE TO: " + pLog.getmGlobalId());
        }
    }

    /**
     * Opens the queue
     */
    public void start() {
        System.err.println("PASSED: STARTING!!");
        mIsAcceptingToQueue = true;
    }

    /**
     * Closes the queue
     */
    public void stop() {
        mIsAcceptingToQueue = false;
    }

    public void addToQueue( Log pLog ) {
        kCSVLoggerQueue.add( pLog );
        if ( !mIsAcceptingToQueue ) {
            System.err.println("FAILED: CANNOT ADD TO QUEUE");
        }
    }

}