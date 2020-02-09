package us.ilite.robot;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.Settings;

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

    public CSVLogger( ) {
        mCSVWriters = new ArrayList<>();
        for ( RobotCodex c : Robot.DATA.mLoggedCodexes ) {
            mCSVWriters.add( new CSVWriter( c ) );
        }
        mIsAcceptingToQueue = false;
        logFromCodexToCSVHeader();
        scheduledFuture = mExService.scheduleAtFixedRate(this::run, Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
    }

    private void run() {
        if ( !kCSVLoggerQueue.isEmpty() ) {
            try {
                ArrayList<Log> kTempCSVLogs = new ArrayList<>();
                kCSVLoggerQueue.drainTo(kTempCSVLogs);
                //mLogger.error( "Drained queue got: " + kTempCSVLogs.size() );

                for ( Log log : kTempCSVLogs ) {
                    logFromCodexToCSVLog( log );
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
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
        if ( mIsAcceptingToQueue ) {
            kCSVLoggerQueue.add( pLog );
        }
    }

    /**
     * Closes all the writers in mNetworkTableWriters
     */
    public void closeWriters() {
        mCSVWriters.forEach(c -> c.closeWriter());
    }

}