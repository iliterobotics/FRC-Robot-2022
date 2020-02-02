package us.ilite.robot;

import edu.wpi.first.wpilibj.Notifier;
import us.ilite.common.CSVLoggerQueue;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;

import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class CSVLogger implements Runnable {
    private Notifier mLoggingNotifier;
    private Data mData;
    private ScheduledExecutorService mExService;
    private JFrame aFrame;
    private JPanel myPanel;
    private JTextField aField;

    public CSVLogger( Data pData ) {
        mData = pData;
//        mLoggingNotifier = new Notifier( this );

        aFrame = new JFrame();
        myPanel = new JPanel(new BorderLayout());
        aField = new JTextField(100);
        myPanel.add(aField, BorderLayout.NORTH);
    }


    @Override
    public void run() {
        mExService.scheduleAtFixedRate(()->{
            System.out.println("Running time is: " + System.currentTimeMillis());
            List<String> kTempCSVLogs = new ArrayList<>();
            System.out.println("Beginning to drain!");
            CSVLoggerQueue.kCSVLoggerQueue.drainTo(kTempCSVLogs);
            System.out.println("Finished draining, got: " + kTempCSVLogs.size());

//            aField.setText( stringThing( kTempCSVLogs ) );

            if ( !kTempCSVLogs.isEmpty() ) {
                try {
                    File file = new File( "us/ilite/robot/logs.txt" );
                    Files.write(file.toPath(), kTempCSVLogs);
                } catch ( Exception e ) {
                    e.printStackTrace();
                }
            }

            //Okay, don't get too far ahead of yourself, uncomment this when ready
            //kTempCSVLogs.stream().forEach(CodexCsvLogger::log);
        }, 5,5, TimeUnit.SECONDS);
    }

    public String stringThing( List<String> arr ) {
        StringBuilder s = new StringBuilder();
        for ( String temp : arr ) {
            s.append(temp);
        }
        return s.toString();
    }

    /**
     * Initiates the Executor Service
     */
    public void start() {
        mData.logFromCodexToCSVHeader();
        mExService = Executors.newSingleThreadScheduledExecutor();
        mExService.schedule( this , Settings.kSecondsToUpdateCSVLogger, TimeUnit.SECONDS);
    }

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