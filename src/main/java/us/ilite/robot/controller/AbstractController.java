package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.*;


import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.drive.EDriveData.*;


import us.ilite.common.types.EFeederData;
import us.ilite.common.lib.util.XorLatch;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

import static us.ilite.robot.Enums.*;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;
    private boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;

    private boolean mIsBallAdded = false;
    private boolean mIsBallOut = false;
    private int mNumBalls = 0;

    public AbstractController(){
        super();
    }


    public void update(){
        if(mEnabled) {
            // split this out so we can put additional common elements here
            updateImpl();

            // Every 10s or so
            mCycleCount++;
        }
        mLastTime = clock.now();
    }

    /**
     * Enables / Disables this controller.
     * @param pEnabled TRUE if enabled
     */
    public final void setEnabled(boolean pEnabled) {
        mCycleCount = 0;
        mEnabled = pEnabled;
    }

    protected abstract void updateImpl();

    protected void updateBalls() {
        if (db.feeder.get(EFeederData.ENTRY_BEAM) == 1d) {
            if (!mIsBallAdded) {
                mNumBalls++;
                mIsBallAdded = true;
            }
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0.2);
        } else if (mIsBallAdded && db.feeder.get(EFeederData.ENTRY_BEAM) == 0d) {
            mIsBallAdded = false;
        }
        else if (db.feeder.get(EFeederData.EXIT_BEAM) == 1d) {
            if (!mIsBallOut) {
                mNumBalls--;
                mIsBallOut = true;
            }
        } else if (mIsBallOut && db.feeder.get(EFeederData.EXIT_BEAM) == 0d) {
            mIsBallOut = false;
        }
        db.feeder.set(EFeederData.NUM_BALLS, mNumBalls);
    }

    /**
     * Provides a way to report on what is used in our codex vs not used. This should help reduce the
     * amount of raw null values we log.
     * @return
     */
    public StringBuilder getUnusedCodexReport() {
        StringBuilder unused = new StringBuilder();
        for(RobotCodex rc : db.mMappedCodex.values()) {
            StringBuilder sb = new StringBuilder();
            int count = 0;
            List<Enum> enums = rc.meta().enums();
            for(Enum e : enums) {
                if(rc.isNull(e)) {
                    sb.append(e.name()).append(",");
                    count++;
                }
            }
            if(count > 0 ) {
                unused.append("Codex ").
                        append(rc.meta().getEnum().getSimpleName()).
                        append(" has empty values at ").
                        append(sb).
                        append("\n");
            }
        }
        return unused;
    }

}