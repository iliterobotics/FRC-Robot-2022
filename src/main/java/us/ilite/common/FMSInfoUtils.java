package us.ilite.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FMSInfoUtils {

    public static final String DEFAULT_EVENT_NAME = "UNKNOWN_EVENT";
    public static final int DEFAULT_MATCH_NUMBER = -1;

    private final NetworkTable fmsTable;

    public String getEventName() {
        String eventName = fmsTable.getEntry("EventName").getString(DEFAULT_EVENT_NAME);
        if(eventName == null || eventName.isEmpty()) {
            eventName = DEFAULT_EVENT_NAME;
        }
        return eventName;
    }

    public int getMatchNumber() {
        int returnedMatchNum = DEFAULT_MATCH_NUMBER;
        Number matchNumber = fmsTable.getEntry("MatchNumber").getNumber(DEFAULT_MATCH_NUMBER);
        if(matchNumber != null) {
            returnedMatchNum = matchNumber.intValue();
        }
        return returnedMatchNum;
    }

    public static FMSInfoUtils getInstance() {
        return INSTANCE_HOLDER.sInstance;
    }

    private FMSInfoUtils() {
        fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
    }

    private static final class INSTANCE_HOLDER {
        private static final FMSInfoUtils sInstance = new FMSInfoUtils();
    }
}
