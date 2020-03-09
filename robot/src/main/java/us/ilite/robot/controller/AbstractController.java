package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.*;

import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.ELimelightData;
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

    /**
     */
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
     * TODO - adjust arm angles
     * @param pEnabled if TRUE, the arm is put down and rollers activated; if FALSE, it is stowed
     */
    protected void setIntakeArmEnabled(boolean pEnabled) {
        if(pEnabled) {
            double speed = Math.max(Math.abs(db.drivetrain.get(L_ACTUAL_VEL_FT_s)), Math.abs(db.drivetrain.get(R_ACTUAL_VEL_FT_s)));
//            if(speed <= 1.0) {
//                speed = 3.0;
//            }
            speed += 3.0;
            db.powercell.set(INTAKE_STATE, EArmState.OUT);
            db.powercell.set(SET_INTAKE_VEL_ft_s , (speed + 1.0)/2.0);
        } else {
            db.powercell.set(INTAKE_STATE, EArmState.STOW);
            db.powercell.set(SET_INTAKE_VEL_ft_s, 0.0);
        }
    }

    protected final void resetSerializerState() {
        mEntryLatch.reset();
        mSecondaryLatch.reset();
        mNumBalls = 0;
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

    protected void reverseSerializer() {
        db.powercell.set(SET_H_pct, -1.0);
        db.powercell.set(SET_V_pct, -0.5);
        db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP,-0.75);
    }

    protected void stopDrivetrain() {
        db.drivetrain.set(STATE, EDriveState.PERCENT_OUTPUT);
        db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
        db.drivetrain.set(DESIRED_TURN_PCT,0.0);
    }

    /**
     * Enables / Disables this controller.
     * @param pEnabled TRUE if enabled
     */
    public final void setEnabled(boolean pEnabled) {
        mCycleCount = 0;
        mEnabled = pEnabled;
    }

    /**
     * Set the flywheel to closed loop and go to target speeds/angles for all relevant
     * sub systems. These speeds/angles are robot-relative and human-readable.
     * @param pSpeed
     */
    protected final void setFlywheelClosedLoop(FlywheelSpeeds pSpeed, boolean pSetHoodState) {
        db.flywheel.set(FLYWHEEL_WHEEL_STATE, pSpeed.wheelstate);
        db.flywheel.set(SET_BALL_VELOCITY_ft_s, pSpeed.speed);
        if (pSetHoodState) {
            setHood(pSpeed);
        }
    }

    protected final void setHood(FlywheelSpeeds pSpeed) {
        db.flywheel.set(HOOD_STATE, pSpeed.hoodstate);
        db.flywheel.set(TARGET_HOOD_ANGLE, pSpeed.angle);
    }

    protected final void setFeederClosedLoop(Enums.FlywheelSpeeds pFlywheelSpeed) {
        db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP, pFlywheelSpeed.feeder);
        db.flywheel.set(SET_FEEDER_rpm, pFlywheelSpeed.feeder * 11000.0);
    }

    protected final void setTurretHandling(TurretControlType pTurretControlType) {
        setTurretHandling(pTurretControlType, Limelight.NONE.id());
    }

    protected final void setTurretHandling(TurretControlType pTurretControlType, int pTrackingId) {
        db.goaltracking.set(ELimelightData.TARGET_ID, pTrackingId);
        db.flywheel.set(EShooterSystemData.TURRET_CONTROL, pTurretControlType);
    }

    private boolean mIsTurretReversed = false;
    protected final void reverseTurretHome() {
        mIsTurretReversed = !mIsTurretReversed;
        db.flywheel.set(HOME_REVERSED, mIsTurretReversed);
    }

    protected void
    firingSequence(FlywheelSpeeds speed, Field2020.FieldElement trackedElement) {
        setHood(speed);
        setFlywheelClosedLoop(speed, false);
        if (isHoodAtCorrectAngle(speed)) {
            setTurretHandling(TurretControlType.TARGET_LOCKING, trackedElement.id());
            if (isTurretAtCorrectAngle() && isFlywheelUpToSpeed()) {
                setFeederClosedLoop(speed);
                if (isFeederUpToSpeed()) {
                    db.powercell.set(SET_V_pct, 0.5);
                    db.powercell.set(SET_H_pct, 0.5);
                } else {
                    db.powercell.set(SET_V_pct, 0);
                    db.powercell.set(SET_H_pct, 0);
                }
            }
        }
    }

    protected void firingSequence(FlywheelSpeeds speed) {
        setFlywheelClosedLoop(speed, true);
        if (isHoodAtCorrectAngle(speed) && isFlywheelUpToSpeed()) {
            db.flywheel.set(SET_FEEDER_rpm, speed.feeder);
            if (isFeederUpToSpeed()) {
                db.powercell.set(SET_V_pct, 0.5);
                db.powercell.set(SET_H_pct, 0.5);
            } else {
                db.powercell.set(SET_V_pct, 0);
                db.powercell.set(SET_H_pct, 0);
            }

        }
    }

    protected boolean isFlywheelUpToSpeed() {
        boolean result = db.flywheel.get(BALL_VELOCITY_ft_s) > 0.0 &&
                db.flywheel.get(BALL_VELOCITY_ft_s) >= db.flywheel.get(SET_BALL_VELOCITY_ft_s) - 2.0;
        SmartDashboard.putBoolean("FLYWHEEL", result);
        return result;
    }

    protected boolean isFeederUpToSpeed() {
        boolean result = db.flywheel.get(SET_FEEDER_rpm) > 0.0 &&
                db.flywheel.get(FEEDER_rpm) >= db.flywheel.get(SET_FEEDER_rpm)*0.8;
        SmartDashboard.putBoolean("FEEDER", result);
        return result;
    }

    protected boolean isTurretAtCorrectAngle(){
        boolean result = db.flywheel.isSet(IS_TARGET_LOCKED);
        SmartDashboard.putBoolean("TURRET", result);
//        TurretControlType turretControlType = db.flywheel.get(TURRET_CONTROL, TurretControlType.class);
//        if (turretControlType != TurretControlType.MANUAL) {
            return result;
//        return true;
//        }
//        return true;
    }
    protected boolean isHoodAtCorrectAngle(FlywheelSpeeds pSpeed) {
        boolean result = Math.abs(db.flywheel.get(CURRENT_HOOD_ANGLE) - pSpeed.angle) <= 1.0;
        SmartDashboard.putBoolean("HOOD", result);
        return result;
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