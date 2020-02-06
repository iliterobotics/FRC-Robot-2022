package us.ilite.common;

import java.util.Objects;

public class Log {
    private Class<Enum> mCodexIdentifier;
    private String mData;

    @Override
    public String toString() {
        return "Log{" +
                "mCodexIdentifier='" + mCodexIdentifier + '\'' +
                ", mData='" + mData + '\'' +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Log log = (Log) o;
        return Objects.equals(mCodexIdentifier, log.mCodexIdentifier) &&
                Objects.equals(mData, log.mData);
    }

    @Override
    public int hashCode() {
        return Objects.hash(mCodexIdentifier, mData);
    }

    public String getmCodexIdentifier() {
        return mCodexIdentifier;
    }

    public void setmCodexIdentifier(String mCodexIdentifier) {
        this.mCodexIdentifier = mCodexIdentifier;
    }

    public String getmData() {
        return mData;
    }

    public void setmData(String mData) {
        this.mData = mData;
    }

    public Log(String mCodexIdentifier, String mData) {
        this.mCodexIdentifier = mCodexIdentifier;
        this.mData = mData;
    }
}
