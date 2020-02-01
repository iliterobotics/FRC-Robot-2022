package us.ilite.robot;

import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.RegularTest;

@Category(CriticalTest.class)
public class RobotUnitTest {

    @Test
    public void testTestController() {
    }
    @Test
    @Category(RegularTest.class)
    public void testRegular() {

    }

}
