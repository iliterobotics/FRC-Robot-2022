package us.ilite.robot;

import com.flybotix.hfr.codex.RobotCodex;
import org.junit.Assert;
import org.junit.Test;
import us.ilite.common.Data;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.controller.AbstractController;
import us.ilite.robot.controller.TestController;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.random;

public class BaseTest {

    protected static final NumberFormat nf = new DecimalFormat("0.00");
    protected static final Data db = Robot.DATA;
    protected static final TestController ctrl = TestController.getInstance();
    static{
        ctrl.setEnabled(true);
    }

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
            pOI.set(pInput, random() * (random() > 0.5d ? 1.0 : -1.0));
        } else {
            pOI.set(pInput, (random() > 0.5 ? 0d : 1d) * (random() > 0.5 ? 1d : -1d));
        }
    }

    protected final void randomizeAllInputs() {
        randomizeDriverInputs();
        randomizeOperatorInputs();
    }

    protected final void commonAssertions(AbstractController pController) {
        testIdempotency(pController);
    }

    /**
     * Tests whether the target controller is Idempotent - i.e. running the controller with the same data twice should
     * net the same exact results.
     * @param pController
     */
    protected final void testIdempotency(AbstractController pController) {
        for(RobotCodex rc : db.mMappedCodex.values()) {
            rc.reset();
        }
        randomizeAllInputs();
        pController.update();
        for(String key : db.mMappedCodex.keySet()) {
            RobotCodex rc = db.mMappedCodex.get(key).copy();
            pController.update();
            String msg = "=== Idempotency test for " + key + " === ";
            Assert.assertEquals(msg, db.mMappedCodex.get(key), rc);
            System.out.println(msg + "PASSED");
        }
    }
}
