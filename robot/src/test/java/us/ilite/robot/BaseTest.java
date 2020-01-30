package us.ilite.robot;

import us.ilite.common.Data;
import us.ilite.common.types.input.ELogitech310;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static java.lang.Math.*;

public class BaseTest {

    protected static final NumberFormat nf = new DecimalFormat("0.00");
    protected static final Data db = Robot.DATA;

    protected final void randomizeDriverInputs() {
        for(ELogitech310 input : ELogitech310.values()) {
            db.driverinput.set(input, random() * random() > 0.5 ? 1d : -1d);
        }
    }

    protected final void randomizeOperatorInputs() {
        for(ELogitech310 input : ELogitech310.values()) {
            db.driverinput.set(input, random() * random() > 0.5 ? 1d : -1d);
        }
    }

    protected final void randomizeAllInputs() {
        randomizeDriverInputs();
        randomizeOperatorInputs();
    }
}
