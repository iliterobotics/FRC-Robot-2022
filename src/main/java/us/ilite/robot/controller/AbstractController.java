package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.*;


import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

import static us.ilite.robot.Enums.*;
import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.EVisionGoal2020.*;



import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;
    private boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;

    private double mStartAngle;


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

    protected void targetAngleLock() {
        if (mStartAngle == 0) {
            mStartAngle = db.drivetrain.get(ACTUAL_TURN_ANGLE_deg);
        }

        double desiredAngle = Math.abs(mStartAngle) <= 90 ? 0 : 180;

        if (db.limelight.isSet(TV)) {
            db.drivetrain.set(STATE, EDriveState.TARGET_ANGLE_LOCK);
        } else {
            db.drivetrain.set(STATE, EDriveState.TURN_TO);
            db.drivetrain.set(DESIRED_TURN_ANGLE_deg, desiredAngle);
        }
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