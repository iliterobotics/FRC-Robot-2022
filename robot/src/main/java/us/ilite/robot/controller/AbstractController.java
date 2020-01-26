package us.ilite.robot.controller;


import us.ilite.common.Data;
import us.ilite.robot.Robot;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;

    public AbstractController(){

        super();
    }

    /**
     * @param pNow the amount of time since the robot started
     */
    public abstract void update(double pNow);

}
