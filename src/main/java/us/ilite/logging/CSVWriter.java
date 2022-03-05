package us.ilite.logging;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ilite.common.FMSInfoUtils;
import us.ilite.common.config.Settings;
import us.ilite.robot.FileUtils;
import us.ilite.robot.Robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Optional;

public class CSVWriter {

    /**
     * The drive where the USB drive is mounted to
     */
    private static final String kUSB_DIR = "u";
    /**
     * The path to the file template
     */
    private static final String kLOG_PATH_FORMAT = "/logs/%s/%s-%s-%s.csv";
    /**
     * The name of the event
     */
    private static final String kEventName = DriverStation.getEventName();
    /**
     * The file writer
     */
    private final Optional<BufferedWriter> optionalWriter;
    /**
     * Logger
     */
    private final ILog mLog = Logger.createLog(CSVWriter.class);
    /**
     * The total number of log fails. Once a value is hit, then we will log an error message
     */
    private static int sLogFailures;
    /**
     * The Codex that is being logged
     */
    private final RobotCodex<?> mCodex;
    /**
     * The CSVLogger responsible for passing the event to this writter
     */
    private final CSVLogger mParentLogger;

    /**
     * Creates the {@link CSVWriter}
     * @param pParentLogger
     *  The parent logger that passes events to this writter
     * @param pCodex
     *  The codex that this writter is handling events for
     */
    CSVWriter(CSVLogger pParentLogger, RobotCodex<?> pCodex) {
        mParentLogger = pParentLogger;
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

    void logCSVLine(String s ) {
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
        mParentLogger.addToQueue( new ImmutablePair<String,RobotCodex>(mCodex.getCSVHeader(),mCodex));
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