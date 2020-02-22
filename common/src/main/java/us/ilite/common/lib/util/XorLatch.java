package us.ilite.common.lib.util;

public class XorLatch {
    public enum State {
        NONE,
        XOR,
        BOTH
    }
    public final Latch mEntry = new Latch();
    public final Latch mExit = new Latch();
    private State mState = State.NONE;

    public XorLatch() {
        mExit.reset();
        mEntry.reset();
    }


    /**
     * Updates latches based upon a single flag's logic. This gives 4 states,
     * two of which are effectively identical:
     * 1.) Flag has not been tripped since reset (pExit.get() && pEntry.get()) == false)
     * 2.) Flag has been tripped but not untripped since reset (pExit.get ^ pEntry.get() == true)
     * 3.) Flag has been tripped and untripped since reset (pEntry.get() && pExit.get() == true)
     * @param pFlag State of the beam breaker
     */
    public void update(boolean pFlag) {
        mEntry.update(mEntry.get() || pFlag);
        // Only trip exit if entry is tripped and the beam is no longer tripped
        mExit.update(mEntry.get() && !pFlag);
        if(mEntry.get() ^ mExit.get()) {
            mState = State.XOR;
        } else if (mExit.get() && mEntry.get()) {
            mState = State.BOTH;
        } else {
            mState = State.NONE;
        }
    }

    public void reset() {
        mEntry.reset();
        mExit.reset();
    }

    public State get() {
        return mState;
    }

}
