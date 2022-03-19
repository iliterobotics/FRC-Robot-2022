package us.ilite.logging;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import us.ilite.common.config.Settings;
import us.ilite.robot.Robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardOpenOption;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import static us.ilite.logging.CSVLogger.kCSVLoggerQueue;

public class CSVWriter {

    public static final String USB_DIR = "/u";
    //public static final String USER_DIR = System.getProperty("user.home");
    private static final String LOG_PATH_FORMAT = "/logs/%s/%s-%s-%s.csv";
    private static String eventName = DriverStation.getInstance().getEventName();
    private final ILog mLog = Logger.createLog(CSVWriter.class);
    private static int mLogFailures;

    private File file;
    private RobotCodex<?> mCodex;

    public CSVWriter(RobotCodex<?> pCodex) {

        mCodex = pCodex;

        mLogFailures = 0;
        file = file();
        if ( file != null ) {
            handleCreation( file );
        }

        if ( eventName.length() <= 0 ) {
            // event name format: MM-DD-YYYY_HH-MM-SS
            eventName =  new SimpleDateFormat("MM-dd-YYYY_HH-mm-ss").format(Calendar.getInstance().getTime());
        }
    }

    public void handleCreation(File pFile) {
        //Makes every folder before the file if the CSV's parent folder doesn't exist
        if(Files.notExists(pFile.toPath())) {
            mLog.error( pFile.getAbsoluteFile().getParentFile().mkdirs() );
        }

        //Creates the .CSV if it doesn't exist
        if(!pFile.exists()) {
            try {
                pFile.createNewFile();
            } catch (IOException e) {}
        }
    }

    public void log( String s ) {
        try {
            if ( file.exists()) {
                Files.writeString(file.toPath(),s+System.lineSeparator(), StandardOpenOption.APPEND);
            }
            else {
                System.err.println("ERROR: FILE: " +file + " does not exist!!");
                if ( mLogFailures < Settings.kAcceptableLogFailures ) {
                    mLog.error("Failure with logging codex: " + mCodex.meta().getEnum().getSimpleName() );
                    mLog.error( "Could not find Path:  (Path to USB)  on roborio! Try plugging in the USB." );
                    mLogFailures++;
                }
                else if ( mLogFailures == Settings.kAcceptableLogFailures ) {
                    mLog.error("---------------------CSV LOGGING DISABLED----------------------");
//                    Robot.mCSVLogger.closeWriters();
                    mLogFailures++;
                }
            }
        } catch (IOException pE) {
            pE.printStackTrace();
        }
    }

    public void writeHeader() {
        kCSVLoggerQueue.add( new Log( mCodex.getCSVHeader(), mCodex.meta().gid() ) );
    }

    public CodexMetadata<?> getMetaDataOfAssociatedCodex() {
        return mCodex.meta();
    }

    public File file() {

        String dir = USB_DIR;

//        if (!dir.isEmpty()) {
        File file = null;
        if (mCodex.meta().getEnum().getSimpleName().equals("ELogitech310")) {
            if (mCodex.meta().gid() == Robot.DATA.driverinput.meta().gid()) {
                file = new File(String.format(dir + LOG_PATH_FORMAT,
                        eventName,
                        "DriverInput",
                        DriverStation.getInstance().getMatchType().name(),
                        DriverStation.getInstance().getMatchNumber()
                ));
            } else {
                file = new File(String.format(dir + LOG_PATH_FORMAT,
                        eventName,
                        "OperatorInput",
                        DriverStation.getInstance().getMatchType().name(),
                        DriverStation.getInstance().getMatchNumber()
                ));
            }
        } else {
            file = new File(String.format(dir + LOG_PATH_FORMAT,
                    eventName,
                    mCodex.meta().getEnum().getSimpleName(),
                    DriverStation.getInstance().getMatchType().name(),
                    DriverStation.getInstance().getMatchNumber()
            ));
        }
        mLog.error("Creating log file at ", file.toPath());

        return file;
    }
}