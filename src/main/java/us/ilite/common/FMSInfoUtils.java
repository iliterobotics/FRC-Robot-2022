package us.ilite.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class responsible for getting the values of the event. This will include information
 * about the overall event itself as well as the match itself. The values all come from
 * {@link NetworkTableInstance}
 */
public class FMSInfoUtils {

    /**
     * Default value of the event if the name is blank or null from the event table
     */
    public static final String DEFAULT_EVENT_NAME = "UNKNOWN_EVENT";
    /**
     * Default value for the match number if the value cannot be retrieved from the event table
     */
    public static final int DEFAULT_MATCH_NUMBER = -1;
    /**
     * {@link NetworkTable} for the fms info itself
     */
    private final NetworkTable fmsTable;

    /**
     * Helper method to get the event name. This method will query the value out of the
     * {@link NetworkTable} for the fms info
     * @return
     *  The name of the event. If this can't be pulled from the {@link NetworkTable} then the
     *  {@link FMSInfoUtils#DEFAULT_EVENT_NAME} will be returned
     */
    public String getEventName() {
        String eventName = fmsTable.getEntry("EventName").getString(DEFAULT_EVENT_NAME);
        if(eventName == null || eventName.isEmpty()) {
            eventName = DEFAULT_EVENT_NAME;
        }
        return eventName;
    }

    /**
     * Helper method to get the match number.
     * @return
     *  The number of the current match. If this value annot be pulled from the {@link NetworkTable} then
     *  the {@link FMSInfoUtils#DEFAULT_MATCH_NUMBER} will be returned
     */
    public int getMatchNumber() {
        int returnedMatchNum = DEFAULT_MATCH_NUMBER;
        Number matchNumber = fmsTable.getEntry("MatchNumber").getNumber(DEFAULT_MATCH_NUMBER);
        if(matchNumber != null) {
            returnedMatchNum = matchNumber.intValue();
        }
        return returnedMatchNum;
    }

    /**
     * Method to get the singleton instance
     * @return
     *  Returns the singleton instance
     */
    public static FMSInfoUtils getInstance() {
        return INSTANCE_HOLDER.sInstance;
    }

    /**
     * Privatee constructor so this doesn't get instantiated more than once.
     */
    private FMSInfoUtils() {
        fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
    }

    /**
     * Instance holder pattern to ensure thread safety in the singleton construction of this class
     */
    private static final class INSTANCE_HOLDER {
        private static final FMSInfoUtils sInstance = new FMSInfoUtils();
    }
}
