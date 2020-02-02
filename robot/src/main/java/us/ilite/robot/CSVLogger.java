package us.ilite.robot;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Notifier;
import us.ilite.common.CSVLoggerQueue;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;

import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class CSVLogger {
    private ILog mLogger = Logger.createLog(this.getClass());
    private Notifier mLoggingNotifier;
    private Data mData;
    private static final ScheduledExecutorService mExService =
            Executors.newSingleThreadScheduledExecutor((run)->new Thread(run, "My timer thread"));
    private ScheduledFuture<?> scheduledFuture;
    //    private JFrame aFrame;
//    private JPanel myPanel;
//    private JTextField aField;

    public CSVLogger( Data pData ) {
        //mLogger.error("CHRIS: CSVLOGGER CONSTRUCTOR");
        mData = pData;
//        mLoggingNotifier = new Notifier( this );

//        aFrame = new JFrame();
//        myPanel = new JPanel(new BorderLayout());
//        aField = new JTextField(100);
//        myPanel.add(aField, BorderLayout.NORTH);
    }

    private void run() {
        try {
            //mLogger.error("CHRIS: RUN!!");
            //System.out.println("Running time is: " + System.currentTimeMillis());
            List<String> kTempCSVLogs = new ArrayList<>();
            //mLogger.error("Beginning to drain!");
            CSVLoggerQueue.kCSVLoggerQueue.drainTo(kTempCSVLogs);
            //mLogger.error("Finished draining, got: " + kTempCSVLogs.size());

            if ( !kTempCSVLogs.isEmpty() ) {
                try {
                    Path path = Paths.get(URI.create("file:///Users/jmz00/Git/Robotics/FileTest.java"));
                    Files.write(path, kTempCSVLogs);
                } catch ( Exception e ) {
                    e.printStackTrace();
                }
            }
            //Okay, don't get too far ahead of yourself, uncomment this when ready
            //kTempCSVLogs.stream().forEach(CodexCsvLogger::log);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }


    /**
     * Initiates the Executor Service
     */
    public void start() {
        //mLogger.error("CHRIS: STARTING THE CSV LOGGER!!");
        mData.logFromCodexToCSVHeader();
        //mLogger.error("CHRIS: BEGINNING TO SCHEDULE");

        scheduledFuture = mExService.scheduleAtFixedRate(this::run, Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
//        mExService.scheduleWithFixedDelay( this , Settings.kSecondsToUpdateCSVLogger, Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
        //mLogger.error("CHRIS: FINISHED TO SCHEDULE");
    }

    /**
     * Stops the periodically called logging by mLoggingNotifier
     */
    public void stop() {
        try {
            if(scheduledFuture != null) {
                //mLogger.error("CHRIS: CANCELING!!");

                scheduledFuture.cancel(true);
                if(true) {
                    throw new RuntimeException("Really?");
                }
                scheduledFuture = null;
            }
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