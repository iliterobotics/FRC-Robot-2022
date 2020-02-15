package us.ilite.robot;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import us.ilite.common.config.Settings;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Optional;

import static us.ilite.robot.CSVLogger.kCSVLoggerQueue;

public class CSVWriter {

    public static final String USB_DIR = "/u";
    //public static final String USER_DIR = System.getProperty("user.home");
    private static final String LOG_PATH_FORMAT = "/logs/%s/%s-%s-%s.csv";
    private static String eventName = DriverStation.getInstance().getEventName();
    private Optional<BufferedWriter> bw;

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

        bw = Optional.empty();
        try  {
            if ( file != null ) {
                bw = Optional.of( new BufferedWriter( new FileWriter( file ) ) );
            }
        } catch ( Exception e ) {
            e.printStackTrace();
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
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void log( String s ) {

        try {
            if ( bw.isPresent() ) {
                bw.get().append(s);
                bw.get().newLine();
            }
            else {
                if ( mLogFailures < Settings.kAcceptableLogFailures ) {
                    mLog.error("Failure with logging codex: " + mCodex.meta().getEnum().getSimpleName() );
                    mLog.error( "Could not find Path:  (Path to USB)  on roborio! Try plugging in the USB." );
                    mLogFailures++;
                    try {
                        file = file();
                        if ( file != null ) {
                            bw = Optional.of( new BufferedWriter( new FileWriter( file ) ) );
                        }
                        else {
                            bw = Optional.empty();
                        }
                    }
                    catch( Exception e ) {
                        e.printStackTrace();
                    }
                }
                else if ( mLogFailures == Settings.kAcceptableLogFailures ) {
                    mLog.error("---------------------CSV LOGGING DISABLED----------------------");
                    mLogFailures++;
                }
            }
        } catch (IOException pE) {
            System.out.println( pE.getLocalizedMessage() );
        }
    }

    public void writeHeader() {
        kCSVLoggerQueue.add( new Log( mCodex.getCSVHeader(), mCodex.meta().gid() ) );
    }

    public CodexMetadata<?> getMetaDataOfAssociatedCodex() {
        return mCodex.meta();
    }

    public File file() {

        // Don't default to home dir to avoid filling up memory
//        String dir = "";
//        if(Files.notExists(new File(USB_DIR).toPath())) {
//            dir = USER_DIR;
//        } else {
//            dir = USB_DIR;
//        }

        //THIS CODE IS USED FOR A MULTI-PARTITION DRIVE AND FINDING IT'S LOCATION IN THE ROBORIO FILE DIRECTORIES
//        String dir = "";
//        char[] letters = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};
//        for ( char c : letters ) {
//            if (Files.exists((new File(String.format("/%c/logs/here.txt", c )).toPath()))) {
//                dir = "/" + c;
//                break;
//            }
//        }

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
//        }

//        mLog.error("Did not make log file for: " + mCodex.meta().getEnum().getSimpleName());
//        return null;
    }
}