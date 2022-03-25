package us.ilite.logging;

import java.util.Objects;

public class Log {
    private String mLogData;
    private int mGlobalId;

    public int getmGlobalId() {
        return mGlobalId;
    }

    public Log( String pLogData, int pGlobalId) {
        mLogData = pLogData;
        mGlobalId = pGlobalId;
    }

    @Override
    public String toString() {
        return "Log{" +
                "mLogData='" + mLogData + '\'' +
                ", mGlobalId=" + mGlobalId +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Log log = (Log) o;
        return mGlobalId == log.mGlobalId &&
                Objects.equals(mLogData, log.mLogData);
    }

    public String getmLogData() {
        return mLogData;
    }

}