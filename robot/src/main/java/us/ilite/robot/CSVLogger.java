package us.ilite.robot;

import edu.wpi.first.wpilibj.Notifier;
import us.ilite.common.CSVLoggerQueue;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class CSVLogger implements Runnable {
    private Notifier mLoggingNotifier;
    private Data mData;
    private ScheduledExecutorService mExService;

    public CSVLogger( Data pData ) {
        mData = pData;
//        mLoggingNotifier = new Notifier( this );
        mData.logFromCodexToCSVHeader();
        mExService = Executors.newSingleThreadScheduledExecutor();
        mExService.schedule( this , Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);

    }


    @Override
    public void run() {
        mExService.scheduleAtFixedRate(()->{
            System.out.println("Running time is: " + System.currentTimeMillis());
            List<String> tempLogStore = new ArrayList<>();
            System.out.println("Beginning to drain!");
            CSVLoggerQueue.kCSVLoggerQueue.drainTo(tempLogStore);
            System.out.println("Finished draining, got: " + tempLogStore.size());

            tempLogStore.stream().forEach(System.out::println);
        }, 5,5, TimeUnit.SECONDS);
    }

//    /**
//     * Starts the periodically called logging by mLoggingNotifier
//     */
//    public void start() {
//        mData.logFromCodexToCSVHeader();
////        mLoggingNotifier.startPeriodic( Settings.kCSVLoggingPeriod );
//    }

    /**
     * Stops the periodically called logging by mLoggingNotifier
     */
    public void stop() {
        try {
            mExService.awaitTermination( Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS );
        }
        catch ( Exception e ) {
            e.printStackTrace();
        }
    }
//
//    public void run() {
//        mData.logFromCodexToCSVLog();
//    }

}