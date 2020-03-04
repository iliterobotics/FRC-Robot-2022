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

import static us.ilite.robot.Enums.*;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
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
     * @param pNow the amount of time since the robot started
     */
    public void update(double pNow){
        dt = pNow - mLastTime;
        if(mEnabled) {
            // split this out so we can put additional common elements here
            updateImpl(pNow);

            // Every 10s or so
            mCycleCount++;
        }
        mLastTime = pNow;
    }

    /**
     * TODO - adjust arm angles
     * @param pNow - Current Timestamp
     * @param pEnabled if TRUE, the arm is put down and rollers activated; if FALSE, it is stowed
     */
    protected void setIntakeArmEnabled(double pNow, boolean pEnabled) {
        if(pEnabled) {
            double speed = Math.max(Math.abs(db.drivetrain.get(L_ACTUAL_VEL_FT_s)), Math.abs(db.drivetrain.get(R_ACTUAL_VEL_FT_s)));
//            if(speed <= 1.0) {
//                speed = 3.0;
//            }
            speed += 3.0;
            db.powercell.set(INTAKE_STATE, EArmState.OUT);
            db.powercell.set(SET_INTAKE_VEL_ft_s, speed + 1);
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
     * @param pNow - current timestamp
     */
    protected void activateSerializer(double pNow) {
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
        SmartDashboard.putString("Entry State", mEntryLatch.get().name());
        SmartDashboard.putString("Secon State", mSecondaryLatch.get().name());
        SmartDashboard.putNumber("# Balls", mNumBalls);
    }

    protected void reverseSerializer(double pNow) {
        db.powercell.set(SET_H_pct, -1.0);
        db.powercell.set(SET_V_pct, -0.5);
        db.flywheel.set(FEEDER_OUTPUT_OPEN_LOOP,-0.75);
    }

    protected void stopDrivetrain(double pNow) {
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
        db.flywheel.set(BALL_VELOCITY_ft_s, pSpeed.speed);
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

    protected void firingSequence(FlywheelSpeeds speed, Field2020.FieldElement trackedElement) {
        setHood(speed);
        setFlywheelClosedLoop(speed, false);
        if (isHoodAtCorrectAngle(speed)) {
            db.goaltracking.set(ELimelightData.TARGET_ID, trackedElement.id());
            db.flywheel.set(EShooterSystemData.TURRET_CONTROL, Enums.TurretControlType.TARGET_LOCKING);
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
        SmartDashboard.putBoolean("Flywheel up to speed", db.flywheel.get(SET_BALL_VELOCITY_ft_s) > 0.0 &&
                db.flywheel.get(SET_BALL_VELOCITY_ft_s) >= db.flywheel.get(BALL_VELOCITY_ft_s) - 2.0);
        return db.flywheel.get(SET_BALL_VELOCITY_ft_s) > 0.0 &&
                db.flywheel.get(SET_BALL_VELOCITY_ft_s) >= db.flywheel.get(BALL_VELOCITY_ft_s) - 2.0;
    }

    protected boolean isFeederUpToSpeed() {
        SmartDashboard.putBoolean("Feeder up to speed", db.flywheel.get(SET_FEEDER_rpm) > 0.0 &&
                db.flywheel.get(FEEDER_rpm) >= db.flywheel.get(SET_FEEDER_rpm)*0.8);
        return db.flywheel.get(SET_FEEDER_rpm) > 0.0 &&
                db.flywheel.get(FEEDER_rpm) >= db.flywheel.get(SET_FEEDER_rpm)*0.8;
    }

    protected boolean isTurretAtCorrectAngle(){
        SmartDashboard.putBoolean("Turret at correct angle", db.flywheel.isSet(IS_TARGET_LOCKED));
//        TurretControlType turretControlType = db.flywheel.get(TURRET_CONTROL, TurretControlType.class);
//        if (turretControlType != TurretControlType.MANUAL) {
//            return db.flywheel.isSet(IS_TARGET_LOCKED);
        return true;
//        }
//        return true;
    }
    protected boolean isHoodAtCorrectAngle(FlywheelSpeeds pSpeed) {
        return Math.abs(db.flywheel.get(CURRENT_HOOD_ANGLE) - pSpeed.angle) <= 1.0;
    }

    protected abstract void updateImpl(double pNow);

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