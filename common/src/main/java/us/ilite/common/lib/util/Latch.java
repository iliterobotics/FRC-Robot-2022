package us.ilite.common.lib.util;

/**
 * Represents a flag that is set exactly one time, until reset() is called
 */
public class Latch {
    private boolean mLatch = false;

    /**
     * @param pFlag updated state / flag
     * @return whether the latch changed from FALSE to TRUE
     */
    public boolean update(boolean pFlag) {
        boolean result = pFlag && !mLatch;
        mLatch = pFlag;
        return result;
    }

    /**
     * @return  current state / flag
     */
    public boolean get() {
        return mLatch;
    }

    /**
     * Unset the latch (set to FALSE)
     */
    public void reset() {
        mLatch = false;
    }
}
