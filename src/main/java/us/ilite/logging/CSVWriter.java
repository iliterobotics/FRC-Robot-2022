package us.ilite.logging;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

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
     * Logger
     */
    private final ILog mLog = Logger.createLog(CSVWriter.class);
    /**
     * The Codex that is being logged
     */
    private final RobotCodex<?> mCodex;
    /**
     * The CSVLogger responsible for passing the event to this writter
     */
    private final CSVLogger mParentLogger;
    /**
     * The logfile itself
     */
    private final File logfile;

    private int logCounter = 0;
    private final NetworkTable csvWriterTable;

    /**
     * Creates the {@link CSVWriter}
     * @param pParentLogger
     *  The parent logger that passes events to this writter
     * @param pCodex
     *  The codex that this writter is handling events for
     */
    CSVWriter(CSVLogger pParentLogger, RobotCodex<?> pCodex) {

        csvWriterTable = NetworkTableInstance.getDefault().getTable("CSVWritter");
        mParentLogger = pParentLogger;
        mCodex = pCodex;

        BufferedWriter bw = null;

        String fileName = getFileName(mCodex);
        logfile = new File(fileName);

        boolean fileCreated = FileUtils.handleCreation(logfile);

        if(!fileCreated) {
            mLog.error("Unable to create logfile: "+fileName +" no logging for codex: " + mCodex.meta().getEnum());
        }

    }

    /**
     * Helper method to write a single line to the file. This will grab a handle to the file
     * and then after it writes the contents to the file, it releases the handle.
     * @param pLine
     *  The line of text to write to the file
     */
    private void write_line(String pLine) {
        logCounter++;
        String logState = "";
        int logFails = -1;
        if(logfile != null && logfile.exists()) {
            try(BufferedWriter bw = new BufferedWriter(new FileWriter(logfile, true))) {
                bw.append(pLine);
                bw.newLine();
                logState = "Log Written-"+System.currentTimeMillis();
            } catch (IOException e){
                mLog.error("Failed to log due to exception");
                mLog.exception(e);
                logState = "Log IOException-"+System.currentTimeMillis();
            }
        } else {
            logState = "Log Failed, log file null or does not exist";
        }

        csvWriterTable.getEntry(mCodex.meta().getEnum().toString()).setString(logCounter + ": "+logState);
    }

    void logCSVLine(String s ) {
        write_line(s);
    }

    public RobotCodex<?> getCodex() {
        return mCodex;
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