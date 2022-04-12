package us.ilite.logging;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

public class FileUtils {
    /**
     * Logger
     */
    private static final ILog mLog = Logger.createLog(CSVWriter.class);

    /**
     * Method to create a file and all of the leading directories.
     * @param pFile
     *  The file to create.
     * @return
     *  true if the file was created without error, false otherwise
     */
    public static boolean handleCreation(File pFile) {
        boolean created = false;
        //Makes every folder before the file if the CSV's parent folder doesn't exist
        if(Files.notExists(pFile.toPath())) {

            boolean mkdirs = pFile.getAbsoluteFile().getParentFile().mkdirs();
            mLog.error("File: " + pFile.toString() +" mkdirs status: " + mkdirs);
        }

        //Creates the .CSV if it doesn't exist
        if(!pFile.exists()) {
            try {
                pFile.createNewFile();
                created = true;
            } catch (IOException e) {
                mLog.exception(e);

            }
        } else {
            created = true;
        }

        return created;
    }

    private FileUtils() {

    }
    private static final class INSTANCE_HOLDER {
        private static final FileUtils sInstance = new FileUtils();
    }
}
