package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.*;

import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import static us.ilite.robot.Enums.*;
import us.ilite.robot.modules.PowerCellModule;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    private boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;


    public static double kIntakeRollerPower_on = 0.5;
    public static double kIntakeRollerPower_off = 0.0;

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
            double speed = Math.max(db.drivetrain.get(L_ACTUAL_VEL_FT_s), db.drivetrain.get(R_ACTUAL_VEL_FT_s));
            if(speed <= 1.0) {
                speed = 0.3;
            }
            db.powercell.set(INTAKE_STATE, EArmState.OUT);
            db.powercell.set(DESIRED_INTAKE_VELOCITY_FT_S, kIntakeRollerPower_on);
        } else {
            db.powercell.set(INTAKE_STATE, EArmState.STOW);
            db.powercell.set(DESIRED_INTAKE_VELOCITY_FT_S, kIntakeRollerPower_off);
        }
    }

    /**
     * Activates the serializer based upon the beam breaks. Assumes all other conditions are met.
     * @param pNow
     * @return
     */
    protected boolean activateSerializer(double pNow) {
        if (db.powercell.isSet(ENTRY_BEAM)) {
            db.powercell.set(DESIRED_H_VELOCITY, 0.3);
            db.powercell.set(DESIRED_V_VELOCITY, 0.35);
            return true;
        } else {
            db.powercell.set(DESIRED_H_VELOCITY, 0.0);
            db.powercell.set(DESIRED_V_VELOCITY, 0.0);
            return false;
        }
    }

    protected void reverseSerializer(double pNow) {
        db.powercell.set(DESIRED_H_VELOCITY, -1.0);
        db.powercell.set(DESIRED_V_VELOCITY, -1.0);
    }

    protected void stopDrivetrain(double pNow) {
        db.drivetrain.set(EDriveData.STATE, EDriveState.PERCENT_OUTPUT);
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
    protected final void setFlywheelClosedLoop(Enums.FlywheelSpeeds pSpeed) {
        db.flywheel.set(FLYWHEEL_WHEEL_STATE, pSpeed.wheelstate);
        db.flywheel.set(TARGET_BALL_VELOCITY, pSpeed.speed);
        db.flywheel.set(HOOD_STATE, pSpeed.hoodstate);
        db.flywheel.set(TARGET_HOOD_ANGLE, pSpeed.angle);
    }


    protected boolean isFlywheelUpToSpeed() {
        return db.flywheel.get(TARGET_BALL_VELOCITY) > 0.0 &&
                db.flywheel.get(CURRENT_BALL_VELOCITY) >= db.flywheel.get(TARGET_BALL_VELOCITY) - 1.0;
    }

    protected boolean isFeederUpToSpeed() {
        return db.flywheel.get(TARGET_FEEDER_VELOCITY_RPM) > 0.0 &&
                db.flywheel.get(CURRENT_FEEDER_VELOCITY_RPM) >= db.flywheel.get(TARGET_FEEDER_VELOCITY_RPM)*0.9;
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
