package us.ilite.robot.controller;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexOf;
import com.flybotix.hfr.util.lang.EnumUtils;
import us.ilite.common.*;
import us.ilite.robot.Robot;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;

    public static <E extends Enum<E> & CodexOf<Double>> E valueOf(Codex<Double, E> data, E pElement) {
        return EnumUtils.getEnums((Class<E>)pElement.getClass(),true).get((int)(double)data.get(pElement));
    }

    public AbstractController(){

        super();
    }

    /**
     * @param pNow the amount of time since the robot started
     */
    public abstract void update(double pNow);

}
