package us.ilite.logging;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.PrintWriter;
import java.io.StringWriter;

public class Logger {

    private ELevel level;


    public Logger(Class<?>loggerClass, ELevel defautLevel) {
        level = defautLevel;
        SmartDashboard.putString("Logger-"+loggerClass,defautLevel.name());

        NetworkTableInstance.getDefault().getTable("SmartDashboard").addEntryListener(new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                if(key.equals("Logger-"+loggerClass)) {
                    level = ELevel.getForName(value.getValue().toString());
                }
            }
        },EntryListenerFlags.kNew|EntryListenerFlags.kUpdate);
    }

    public void debug(String message) {
        debug(message, null);
    }
    public void debug(String message, Throwable ex) {
        if(level == ELevel.DEBUG) {
            log("DEBUG: " + message,ex,false);
        }
    }

    public void warn(String message) {
        warn(message, null);
    }
    public void warn(String message, Throwable ex) {
        if(level.ordinal() <= ELevel.WARN.ordinal()) {
            log("WARN: "+message,ex,false);
        }
    }
    public void error(String error) {
        error(error,null);

    }

    public void error(String error, Throwable ex) {
        log("ERROR: " + error, ex,true);
    }

    private void log(String message, Throwable ex, boolean error) {
        String exception = ex != null ? getExceptionString(ex) : "";

        if(error) {
            System.err.println(message+" " + exception);
        } else {
            System.out.println(message+" " +exception);
        }
    }


    public void exception(Throwable ex) {
        System.err.println("ERROR: " + getExceptionString(ex));
    }

    private String getExceptionString(Throwable ex) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        ex.printStackTrace(pw);

        return sw.toString();
    }
}
