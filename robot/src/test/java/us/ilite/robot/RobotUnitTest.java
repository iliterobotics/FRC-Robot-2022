package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexOf;
import com.flybotix.hfr.util.lang.EnumUtils;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.RegularTest;
import us.ilite.common.Data;
import us.ilite.common.types.EColorData;
import us.ilite.robot.controller.AbstractController;
import us.ilite.robot.controller.TestController;
import us.ilite.robot.modules.DJSpinnerModule;

@Category(CriticalTest.class)
public class RobotUnitTest extends BaseTest{

    @Test
    public void testTestController() {
    }
    @Test
    @Category(RegularTest.class)
    public void testRegular() {

    }

    @Test
    @Category(CriticalTest.class)
    public void testCodexValueOf() {
        TestController t = new TestController();
        randomizeAllInputs();
        Data.setState(db.color, EColorData.SENSED_COLOR, DJSpinnerModule.EColorMatch.BLUE);
        System.out.println(db.driverinput);
        DJSpinnerModule.EColorMatch c = Data.valueOf(db.color, DJSpinnerModule.EColorMatch.class, EColorData.SENSED_COLOR);
        System.out.println(c);

    }

}
