package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.*;


import static us.ilite.common.types.EFeederData.*;
import static us.ilite.common.types.EIntakeData.DESIRED_ROLLER_pct;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

import static us.ilite.robot.Enums.*;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;
    protected boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;

    protected boolean mIsBallAdded = false;
    protected boolean mIsBallOut = false;
    protected int mNumBalls = 0;
    protected int mBallsShot = 0;
    protected XorLatch mEntryGate  = new XorLatch();
    protected XorLatch mExitGate = new XorLatch();
    protected Timer mShotTimer = new Timer();
    protected boolean mFireWanted = false;

    //Was 5000, lowered it to 4500 before match 42.
    //After match 42, we lowered it even further by 1000rpm
    protected double mFeederFireSpeed = 3500d;

    public AbstractController(){
        super();
    }


    public void update(){
        if(mEnabled) {
            // split this out so we can put additional common elements here
            updateImpl();
            mExitGate.update(db.feeder.isSet(EXIT_BEAM));
            mEntryGate.update(db.feeder.isSet(ENTRY_BEAM));
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
        }
        db.feeder.set(NUM_BALLS, mNumBalls);
    }

    protected boolean climberWithinTolerance(double pTolerance) {
        Enums.EClimberAngle current = db.climber.get(EClimberData.CURRENT_RUNG, Enums.EClimberAngle.class);
        Enums.EClimberAngle desired = db.climber.get(EClimberData.DESIRED_RUNG, Enums.EClimberAngle.class);
        return Math.abs(current.getAngle() - desired.getAngle()) < pTolerance;
    }

    protected boolean climberWithinTolerance(double pTolerance, double currentAngle, Enums.EClimberAngle pAngle) {
        return Math.abs((currentAngle) - pAngle.getAngle()) < pTolerance;
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
        db.intake.set(EIntakeData.ROLLER_STATE, ERollerState.PERCENT_OUTPUT);
        if (enabled) {
            db.intake.set(EIntakeData.ARM_STATE, EArmState.EXTEND);
        } else {
            db.intake.set(DESIRED_ROLLER_pct, 0.0);
            db.intake.set(EIntakeData.ARM_STATE, EArmState.RETRACT);
        }
    }

    protected abstract void updateImpl();

    protected void fireCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.VELOCITY);
        db.feeder.set(EFeederData.SET_VELOCITY_rpm, mFeederFireSpeed);
        setLED(LEDColorMode.DEFAULT, LEDState.SOLID);
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
        //Removed exit beam
//        //Indexing balls coming out
//        //Got rid of else if here
//        if (db.feeder.get(EXIT_BEAM) == 0d) {
//            if (!mIsBallOut) {
//                mNumBalls--;
//                mIsBallOut = true;
//            }
//        } else if (mIsBallOut && db.feeder.get(EXIT_BEAM) == 1d) {
//            mIsBallOut = false;
//        }
        db.feeder.set(NUM_BALLS, mNumBalls);
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
        mNumBalls = 0;
    }

    protected void intakeCargo() {
        setIntakeArmEnabled(true);
        db.intake.set(EIntakeData.ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
        db.intake.set(EIntakeData.DESIRED_ROLLER_pct, 1.0);
        indexCargo();
//        activateFeeder();
    }

    protected void reverseCargo() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.feeder.set(SET_FEEDER_pct, -1.0);
        mNumBalls = 0;
        db.intake.set(DESIRED_ROLLER_pct, -1.0);
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
    public void activateFeeder() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        //Entry beam has been tripped and has not been untripped (ball has just hit beam break but not entered system)
        if (mEntryGate.get() == XorLatch.State.XOR)  {
            db.feeder.set(SET_FEEDER_pct, 0.4);
            mNumBalls++;
        }
        //Cargo has past the entrance and hasn't reached the exit beam yet and another has entered
        else if (mEntryGate.get() == XorLatch.State.BOTH && mExitGate.get() == XorLatch.State.NONE
                && mNumBalls == 2) {
            db.feeder.set(SET_FEEDER_pct, 0.4);
            mEntryGate.reset();
        }
        //Cargo has past the entrance however there is only one ball so don't do anything yet
        else if (mEntryGate.get() == XorLatch.State.BOTH && mExitGate.get() == XorLatch.State.NONE
                && mNumBalls == 1) {
            db.feeder.set(SET_FEEDER_pct, 0.0);
            mEntryGate.reset();
        }
        //This only happens when there is one ball and another ball has not entered
        // OR the beam was never tripped in the first place
        else {
            db.feeder.set(SET_FEEDER_pct, 0.0);
        }
        //Stop indexing if we have hit the exit beam (make sure no balls get shot out prematurely)
        if (mExitGate.get() == XorLatch.State.XOR) {
            db.feeder.set(SET_FEEDER_pct, 0.0);
        }
    }

    public void stageFeeder() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        if (mExitGate.get() == XorLatch.State.NONE) {
            db.feeder.set(SET_FEEDER_pct, 0.4);
        } else if (mExitGate.get() == XorLatch.State.XOR) {
            db.feeder.set(SET_FEEDER_pct, 0.0);
        }
    }

    public void fireFeeder(double pSpeed, double pPulseSpeed) {
        db.feeder.set(EFeederData.STATE, EFeederState.VELOCITY);
        setIntakeArmEnabled(false);
        if (mExitGate.get() == XorLatch.State.XOR) {
            mShotTimer.start();
            //If there haven't been any balls shot during the fire period go ahead and fire
            // (no collision is going to occur)
            if (mBallsShot == 0) {
                db.feeder.set(SET_VELOCITY_rpm, pSpeed);
            }
            //Otherwise, make sure that you don't potentially hit any other ball
            else {
                if (mShotTimer.get() % pPulseSpeed == 0) {
                    db.feeder.set(SET_VELOCITY_rpm, pSpeed);
                }
                else {
                    db.feeder.set(SET_FEEDER_pct, 0.0);
                }
            }
        }
        else if (db.feeder.get(EXIT_BEAM) == 1) {
            mExitGate.reset();
            mBallsShot++;
            mNumBalls--;
        }
    }
    public void placeFeeder() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.intake.set(EIntakeData.ROLLER_STATE, ERollerState.PERCENT_OUTPUT);
        db.feeder.set(SET_FEEDER_pct, -0.2);
        db.intake.set(EIntakeData.DESIRED_ROLLER_pct, -0.1);
        if (mEntryGate.get() == XorLatch.State.BOTH) {
            mEntryGate.reset();
            mNumBalls--;
        }
    }
    public void reverseFeeder() {
        db.feeder.set(EFeederData.STATE, EFeederState.PERCENT_OUTPUT);
        db.intake.set(EIntakeData.ROLLER_STATE, ERollerState.PERCENT_OUTPUT);
        db.feeder.set(SET_FEEDER_pct, -1.0);
        db.intake.set(DESIRED_ROLLER_pct, -1.0);
        if (mEntryGate.get() == XorLatch.State.BOTH) {
            mEntryGate.reset();
            mNumBalls--;
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