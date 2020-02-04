package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import us.ilite.common.*;
import us.ilite.robot.Robot;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    private boolean mEnabled = false;

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
        }
    }

    public final void setEnabled(boolean pEnabled) {
        mEnabled = pEnabled;
    }

    protected abstract void updateImpl(double pNow);
}
