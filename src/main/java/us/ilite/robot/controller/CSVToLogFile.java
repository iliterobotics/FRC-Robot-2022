package us.ilite.robot.controller;

import java.util.Collection;

public class CSVToLogFile {

    public static CSVToLogFile getInstance() {
        return INSTANCE_HOLDER.sInstance;
    }

    public void logCSVData(Collection<?>data, Class<?>logClass) {
        StringBuilder str = new StringBuilder();

        boolean first = true;
        for(Object anObj : data) {
            if(!first) {
                str.append(",");
            }

            str.append(anObj);
            first = false;
        }

        System.out.println(logClass.toString() +": " + str.toString());
    }

    private static final class INSTANCE_HOLDER {



        private static final CSVToLogFile sInstance = new CSVToLogFile();
    }
}
