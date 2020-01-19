package us.ilite.robot.controller;


public abstract class AbstractController {

    public AbstractController(){

        super();
    }

    /**
     * @param pNow the amount of time since the robot started
     */
    public abstract void update(double pNow);

}
