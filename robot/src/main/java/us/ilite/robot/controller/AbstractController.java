package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import us.ilite.common.*;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Utils;
import us.ilite.common.types.EMatchMode;
import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    private boolean mEnabled = false;
    protected int mCycleCount = 0;

    public AbstractController(){

        super();
    }

    /**
     * @param pNow the amount of time since the robot started
     */
    public void update(double pNow){
        if(mEnabled) {
            // split this out so we can put additional common elements here
            updateImpl(pNow);

            // Every 10s or so
            mCycleCount++;
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
