package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.*;


import static us.ilite.common.types.drive.EDriveData.*;


import us.ilite.common.types.EFeederData;
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
    private boolean isBallAdded = false;
    private boolean isBallOut = false;
    private int numBalls = 0;

    protected int mNumBalls = 0;

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

    public void indexBalls() {
        // WHATEVER
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
        db.drivetrain.set(STATE, EDriveState.PERCENT_OUTPUT);
        db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
        db.drivetrain.set(DESIRED_TURN_PCT,0.0);
    }

    protected abstract void updateImpl();

    protected void indexCargo() {
        //Indexing balls coming in
        if (db.feeder.get(EFeederData.ENTRY_BEAM) == 1d) {
            if(!isBallAdded) {
                numBalls++;
                isBallAdded = true;
            }
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
        } else if (isBallAdded && db.feeder.get(EFeederData.ENTRY_BEAM) == 0d) {
            isBallAdded = false;
        }
        //Indexing balls coming out
        else if (db.feeder.get(EFeederData.EXIT_BEAM) == 1d) {
            db.feeder.set(EFeederData.SET_CONVEYOR_pct, 0.2);
            if(!isBallOut) {
                numBalls--;
                isBallOut = true;
            }
        } else if (isBallOut && db.feeder.get(EFeederData.EXIT_BEAM) == 0d) {
            isBallOut = false;
        }
        db.feeder.set(EFeederData.NUM_BALLS, numBalls);
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