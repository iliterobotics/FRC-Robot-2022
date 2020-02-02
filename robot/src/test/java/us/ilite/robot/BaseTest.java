package us.ilite.robot;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.Data;
import us.ilite.common.types.input.ELogitech310;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static java.lang.Math.random;

public class BaseTest {

    protected static final NumberFormat nf = new DecimalFormat("0.00");
    protected static final Data db = Robot.DATA;

    protected final void randomizeDriverInputs() {
        for(ELogitech310 input : ELogitech310.values()) {
            setInputToRandom(db.driverinput, input);
        }
    }

    protected final void randomizeOperatorInputs() {
        for(ELogitech310 input : ELogitech310.values()) {
            setInputToRandom(db.operatorinput, input);
        }
    }

    private final void setInputToRandom(RobotCodex<ELogitech310> pOI, ELogitech310 pInput) {
        if(pInput.isAxis()) {
            pOI.set(pInput, random() * random() > 0.5 ? 1d : -1d);
        } else {
            pOI.set(pInput, random() > 0.5 ? 0d : 1d * random() > 0.5 ? 1d : -1d);
        }
    }

    protected final void randomizeAllInputs() {
        randomizeDriverInputs();
        randomizeOperatorInputs();
    }
}
