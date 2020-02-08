package us.ilite.robot;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.CSVLoggerQueue;
import us.ilite.common.Data;
import us.ilite.common.Log;
import us.ilite.common.config.Settings;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class CSVLogger {
    private ILog mLogger = Logger.createLog(this.getClass());
    private Data mData;
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "My timer thread"));
    private ScheduledFuture<?> scheduledFuture;

    public CSVLogger(Data pData ) {
        mData = pData;
    }

    private void run() {
        try {
            ArrayList<Log> kTempCSVLogs = new ArrayList<>();
            CSVLoggerQueue.kCSVLoggerQueue.drainTo(kTempCSVLogs);

            for ( Log log : kTempCSVLogs ) {
                mData.logFromCodexToCSVLog( log );
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    /**
     * Initiates the Executor Service
     */
    public void start() {
        mLogger.error("Starting CSV Logging!!!!");
        mData.logFromCodexToCSVHeader();
        scheduledFuture = mExService.scheduleAtFixedRate(this::run, Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
    }

    /**
     * Ends the scheduled executor service when called
     * Keep in mind that robot initialization and process sequencing in the Robot.class is weird and CSV logger must be compatible.
     */
    public void stop() {

        scheduledFuture.cancel(true);
        Robot.DATA.closeWriters();
//        try {
//            if(scheduledFuture != null) {
//                scheduledFuture.cancel(true);
//                if(true) {
//                    throw new RuntimeException("Really?");
//                }
//                scheduledFuture = null;
//            }
//        }
//        catch ( Exception e ) {
//            e.printStackTrace();
//        }
    }

}