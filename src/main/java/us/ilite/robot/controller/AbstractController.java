package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.*;

import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;
import us.ilite.robot.modules.Limelight;

import static us.ilite.robot.Enums.*;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;
    private boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;


    public static double kIntakeRollerPower_on = 0.6;
    public static double kIntakeRollerPower_off = 0.0;
    protected final XorLatch mSecondaryLatch = new XorLatch();
    protected final XorLatch mEntryLatch = new XorLatch();
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


    protected final void resetSerializerState() {
        mEntryLatch.reset();
        mSecondaryLatch.reset();
        mNumBalls = 0;
    }

    protected void stopDrivetrain() {
        db.drivetrain.set(STATE, EDriveState.PERCENT_OUTPUT);
        db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
        db.drivetrain.set(DESIRED_TURN_PCT,0.0);
    }

    /**
     * Activates the serializer based upon the beam breaker states
     */
    protected void activateSerializer() {
        mEntryLatch.update(db.powercell.isSet(ENTRY_BEAM));
        mSecondaryLatch.update(db.powercell.isSet(H_BEAM));
        if(mNumBalls >= 3) {
            if (db.powercell.isSet(ENTRY_BEAM) && mNumBalls < 5) {
                db.powercell.set(SET_H_pct, 0.3);
            } else {
                db.powercell.set(SET_H_pct, 0.0);
                db.powercell.set(SET_V_pct, 0.0);
            }
            if(mEntryLatch.get() == XorLatch.State.BOTH) {
                mNumBalls++;
                mEntryLatch.reset();
            }
        } else {
            if (mSecondaryLatch.get() == XorLatch.State.XOR) {
                // Ball has entered but not exited
                db.powercell.set(SET_H_pct, 0.0);
                db.powercell.set(SET_V_pct, 0.35);
            } else if (mSecondaryLatch.get() == XorLatch.State.NONE) {
                // Ball has not entered
                db.powercell.set(SET_H_pct, 0.25);
                db.powercell.set(SET_V_pct, 0.0);
            } else {
                // Ball has exited
                db.powercell.set(SET_H_pct, 0.0);
                db.powercell.set(SET_V_pct, 0.0);
                mSecondaryLatch.reset();
                mNumBalls++;
            }
        }
        db.powercell.set(NUM_BALLS, mNumBalls);
        db.powercell.set(ENTRY_GATE, mEntryLatch.get());
        db.powercell.set(H_GATE, mSecondaryLatch.get());
    }

    protected abstract void updateImpl();

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