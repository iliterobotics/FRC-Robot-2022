package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.*;


import static us.ilite.common.types.EFeederData.*;
import static us.ilite.common.types.EIntakeData.DESIRED_pct;
import static us.ilite.common.types.drive.EDriveData.*;


import us.ilite.common.config.InputMap;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
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

    protected void stopDrivetrain() {
        db.drivetrain.set(EDriveData.STATE, EDriveState.PERCENT_OUTPUT);
        db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
        db.drivetrain.set(DESIRED_TURN_PCT,0.0);
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
    }

    protected void fireCargo() {
        db.feeder.set(EFeederData.SET_FEEDER_pct, 1d);
        mNumBalls = 0;
        db.feeder.set(NUM_BALLS, 0);
    }

    protected void indexCargo() {
        if (db.feeder.get(RESET_BALLS) == 1d) {
            mNumBalls = 0;
        } else {
            mNumBalls = (int)db.feeder.get(NUM_BALLS);
        }
        if (db.feeder.get(EFeederData.ENTRY_BEAM) == 0d) {
            if (!mIsBallAdded) {
                mNumBalls++;
                mIsBallAdded = true;
            }
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0.4);
        } else if (mIsBallAdded) {
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0d);
            mIsBallAdded = false;
        } else if (mNumBalls > 0) {
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0d);
        } else {
            db.feeder.set(EFeederData.SET_FEEDER_pct, 0d);
        }
        db.feeder.set(EFeederData.NUM_BALLS, mNumBalls);
    }

    protected void placeCargo() {
        db.feeder.set(EFeederData.SET_FEEDER_pct, -0.2);
        db.intake.set(EIntakeData.DESIRED_pct, -0.1);
    }

    protected void reverseCargo() {
        db.feeder.set(SET_FEEDER_pct, -1.0);
        mNumBalls = 0;
        db.feeder.set(EFeederData.NUM_BALLS, 0);
        if (db.intake.get(EIntakeData.PNEUMATIC_STATE) == 1.0) {
            db.intake.set(DESIRED_pct, 0.0);
        } else {
            db.intake.set(DESIRED_pct, -1.0);
        }
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