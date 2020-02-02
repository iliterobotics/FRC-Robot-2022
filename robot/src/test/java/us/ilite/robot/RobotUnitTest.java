package us.ilite.robot;

import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.RegularTest;
import us.ilite.robot.controller.TestController;

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

    }

}
