package us.ilite.logging;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ilite.common.config.Settings;
import us.ilite.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;

public class CSVLogger {
    public static LinkedBlockingDeque<ImmutablePair<String,RobotCodex>> kCSVLoggerQueue = new LinkedBlockingDeque<>();
    private final Map<Integer, CSVWriter>mCSVWriters = new ConcurrentHashMap<>();
    private ILog mLogger = Logger.createLog(this.getClass());
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "My timer thread"));
    private ScheduledFuture<?> scheduledFuture;
    private boolean mIsAcceptingToQueue;

    public CSVLogger( boolean pIsLogging ) {
        if ( pIsLogging ) {
            for (RobotCodex c : Robot.DATA.mLoggedCodexes) {
                mCSVWriters.put(c.meta().gid(),new CSVWriter((c)));
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
                List<ImmutablePair<String,RobotCodex>> kTempCSVLogs = new ArrayList<>();
                kCSVLoggerQueue.drainTo(kTempCSVLogs);
                for ( ImmutablePair<String,RobotCodex> log : kTempCSVLogs ) {
                    logFromCodexToCSVLog( log );
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void logFromCodexToCSVHeader() {
        for(CSVWriter writer : mCSVWriters.values()) {
            writer.writeHeader();
        }

    }
    
    private void logFromCodexToCSVLog( ImmutablePair<String,RobotCodex> pLog ) {
        CSVWriter writer = mCSVWriters.get(pLog.getRight().meta().gid());
        if(writer != null) {
            writer.log(pLog.getLeft());
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

    public void addToQueue( ImmutablePair<String,RobotCodex> pLog ) {
        if ( mIsAcceptingToQueue ) {
            kCSVLoggerQueue.add( pLog );
        }
    }

    public void closeWriters() {
        for(CSVWriter cw : mCSVWriters.values()) {
            cw.close();
        }
    }

}