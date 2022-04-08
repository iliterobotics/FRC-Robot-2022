package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.*;


import static us.ilite.common.types.EFeederData.*;
import static us.ilite.common.types.EIntakeData.DESIRED_ROLLER_pct;
import static us.ilite.common.types.drive.EDriveData.*;


import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
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
    private boolean mIsReverseBallAdded = false;
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
    public void updateBallCount() {
        if (mNumBalls < 0) {
            mNumBalls = 0;
        }
        if (!db.limelight.isSet(ELimelightData.TV)) {
            if (mNumBalls == 0) {
                setLED(LEDColorMode.DEFAULT, Enums.LEDState.SOLID);
            }
            else if (mNumBalls == 1) {
                setLED(LEDColorMode.YELLOW, Enums.LEDState.SOLID);
            }
            else if (mNumBalls == 2) {
                setLED(LEDColorMode.PURPLE, Enums.LEDState.SOLID);
            }
            else {
                setLED(LEDColorMode.ORANGE, LEDState.SOLID);
            }
        }
        db.feeder.set(NUM_BALLS, mNumBalls);
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
        db.drivetrain.set(DESIRED_TURN_PCT, 0.0);
    }

    protected void setIntakeArmEnabled(boolean enabled) {
        if (enabled) {
            db.intake.set(EIntakeData.ARM_STATE, EArmState.EXTEND);
        } else {
            db.intake.set(EIntakeData.ARM_STATE, EArmState.RETRACT);
        }
    }

    protected abstract void updateImpl();

    protected void fireCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.feeder.set(EFeederData.SET_FEEDER_pct, 0.9d);
        setLED(LEDColorMode.DEFAULT, LEDState.SOLID);
        indexCargo();
    }
    protected void indexCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        //Indexing balls coming in
        if (db.feeder.get(ENTRY_BEAM) == 0d) {
            if (!mIsBallAdded) {
                mNumBalls++;
                mIsBallAdded = true;
            }
            db.feeder.set(SET_FEEDER_pct, 0.4);
        } else if (mIsBallAdded && db.feeder.get(ENTRY_BEAM) == 1d) {
            mIsBallAdded = false;
        }
        //Indexing balls coming out
        //Got rid of else if here
        if (db.feeder.get(EXIT_BEAM) == 0d) {
            if (!mIsBallOut) {
                mNumBalls--;
                mIsBallOut = true;
            }
        } else if (mIsBallOut && db.feeder.get(EXIT_BEAM) == 1d) {
            mIsBallOut = false;
        }
        updateBallCount();
    }

    protected void reverseIndex() {
        //Indexing balls leaving
        if (db.feeder.get(ENTRY_BEAM) == 0d) {
            if (!mIsReverseBallAdded) {
                mNumBalls--;
                mIsReverseBallAdded = true;
            }
        } else if (mIsReverseBallAdded && db.feeder.get(ENTRY_BEAM) == 1d) {
            mIsReverseBallAdded = false;
        }
        updateBallCount();
    }

    protected void stageBalls() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        if (db.feeder.get(EXIT_BEAM) == 1d) {
            db.feeder.set(SET_FEEDER_pct, 0.4);
        }
    }

    protected void placeCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.feeder.set(EFeederData.SET_FEEDER_pct, -0.2);
        db.intake.set(EIntakeData.DESIRED_ROLLER_pct, -0.1);
        reverseIndex();
    }

    protected void intakeCargo() {
        setIntakeArmEnabled(true);
        db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
        db.intake.set(EIntakeData.DESIRED_ROLLER_pct, 1.0);
        indexCargo();
    }

    protected void reverseCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.feeder.set(SET_FEEDER_pct, -1.0);
        db.intake.set(DESIRED_ROLLER_pct, -1.0);
        reverseIndex();
    }

    protected void setLED(Enums.LEDColorMode pColor, Enums.LEDState pState) {
        //pState is in seconds
        db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, pColor);
        db.ledcontrol.set(ELEDControlData.LED_STATE, pState);
        if(pState == LEDState.BLINKING) {
            db.ledcontrol.set(ELEDControlData.BLINK_SPEED, 0.5);
        }
    }
    public void setLED(Enums.LEDColorMode pColor, double pBlinkRate) {
        //pState is in seconds
        db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, pColor);
        db.ledcontrol.set(ELEDControlData.LED_STATE, LEDState.BLINKING);
        db.ledcontrol.set(ELEDControlData.BLINK_SPEED,pBlinkRate);
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