package us.ilite.robot;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.CSVLoggerQueue;
import us.ilite.common.Data;
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
            //System.out.println("Running time is: " + System.currentTimeMillis());
            ArrayList<String> kTempCSVLogs = new ArrayList<>();
            //mLogger.error("Beginning to drain!");
            CSVLoggerQueue.kCSVLoggerQueue.drainTo(kTempCSVLogs);
           mLogger.error("Finished draining, got: " + kTempCSVLogs.size());

//            if ( !kTempCSVLogs.isEmpty() ) {
//                try {
//                    Path path = Paths.get(URI.create("file:///Users/jmz00/Git/Robotics/FileTest.java"));
//                    Files.write(path, kTempCSVLogs);
//                } catch ( Exception e ) {
//                    e.printStackTrace();
//                }
//            }

            for ( String logline : kTempCSVLogs ) {
                mData.logFromCodexToCSVLog( logline );
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