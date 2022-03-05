package us.ilite.robot;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ilite.common.FMSInfoUtils;
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

    /**
     * The drive where the USB drive is mounted to
     */
    private static final String kUSB_DIR = "u";
    private static final String kLOG_PATH_FORMAT = "/logs/%s/%s-%s-%s.csv";
    private static final String kEventName = DriverStation.getEventName();
    private final Optional<BufferedWriter> optionalWriter;

    private final ILog mLog = Logger.createLog(CSVWriter.class);
    private static int sLogFailures;

    private RobotCodex<?> mCodex;

    public CSVWriter(RobotCodex<?> pCodex) {
        mCodex = pCodex;

        BufferedWriter bw = null;

        sLogFailures = 0;
        String fileName = getFileName(mCodex);
        File file = new File(fileName);

        boolean fileCreated = FileUtils.handleCreation(file);

        if(fileCreated) {
            try {
                bw = new BufferedWriter( new FileWriter( file ) );
            } catch (IOException e) {
                mLog.exception(e);
            }
        }

        optionalWriter = Optional.ofNullable(bw);

    }

    public void log( String s ) {
        try {

            if ( optionalWriter.isPresent() ) {
                optionalWriter.get().append(s);
                optionalWriter.get().newLine();
            }
            else {
                if ( sLogFailures < Settings.kAcceptableLogFailures ) {
                    mLog.error("Failure with logging codex: " + mCodex.meta().getEnum().getSimpleName() );
                    mLog.error( "Could not find Path:  (Path to USB)  on roborio! Try plugging in the USB." );
                    sLogFailures++;
                }
                else if ( sLogFailures == Settings.kAcceptableLogFailures ) {
                    mLog.error("---------------------CSV LOGGING DISABLED----------------------");
//                    Robot.mCSVLogger.closeWriters();
                    sLogFailures++;
                }
            }
        } catch (IOException pE) {}
    }

    public void close() {
        if ( optionalWriter.isPresent() ) {
            try {
                optionalWriter.get().flush();
                optionalWriter.get().close();
            }
            catch ( Exception e ) {}

        }
    }

    public void writeHeader() {
        kCSVLoggerQueue.add( new ImmutablePair<String,RobotCodex>(mCodex.getCSVHeader(),mCodex));
    }

    public CodexMetadata<?> getMetaDataOfAssociatedCodex() {
        return mCodex.meta();
    }

    private static final String getFileName(RobotCodex<?>pCodex) {
        String fileName = "";
        String logDir = "/"+kUSB_DIR;

        logDir = logDir+"/"+ FMSInfoUtils.getInstance().getEventName()+"/"+FMSInfoUtils.getInstance().getMatchNumber();

        switch(pCodex.meta().getEnum().getSimpleName()) {
            case "ELogitech310":
                if (pCodex.meta().gid() == Robot.DATA.driverinput.meta().gid()) {
                    fileName = String.format(logDir + kLOG_PATH_FORMAT,
                            kEventName,
                            "DriverInput",
                            DriverStation.getMatchType().name(),
                            DriverStation.getMatchNumber());
                } else {
                    fileName = String.format(logDir + kLOG_PATH_FORMAT,
                            kEventName,
                            "DriverInput",
                            DriverStation.getMatchType().name(),
                            DriverStation.getMatchNumber());
                }
            default:
                fileName = String.format(logDir + kLOG_PATH_FORMAT,
                        kEventName,
                        pCodex.meta().getEnum().getSimpleName(),
                        DriverStation.getMatchType().name(),
                        DriverStation.getMatchNumber());
        }

        return fileName;
    }
}