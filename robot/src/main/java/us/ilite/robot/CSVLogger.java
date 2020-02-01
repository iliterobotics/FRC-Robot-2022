package us.ilite.robot;

import edu.wpi.first.wpilibj.Notifier;
import us.ilite.common.*;
import us.ilite.common.config.*;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.*;

public class CSVLogger implements Runnable {
    private Notifier mLoggingNotifier;
    private Data mData;
    private ScheduledExecutorService mExService;

    public CSVLogger( Data pData ) {
        mData = pData;
//        mLoggingNotifier = new Notifier( this );
        mExService = Executors.newSingleThreadScheduledExecutor();
        mExService.schedule(mData::logFromCodexToCSVLog, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
    }


    @Override
    public void run() {
        mExService.scheduleAtFixedRate(()->{
            System.out.println("Running time is: " + System.currentTimeMillis());
            List<String> tempLogStore = new ArrayList<>();
            System.out.println("Beginning to drain!");
            CSVLoggerQueue.kCSVLoggerQueue.;
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
//
//    /**
//     * Stops the periodically called logging by mLoggingNotifier
//     */
//    public void stop() {
//        try {
//            mExService.awaitTermination( Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS );
//        }
//        catch ( Exception e ) {
//            e.printStackTrace();
//        }
//    }
//
//    public void run() {
//        mData.logFromCodexToCSVLog();
//    }

}