package us.ilite.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.RegularTest;
import us.ilite.robot.controller.TestController;

@Category(CriticalTest.class)
public class RobotUnitTest extends BaseTest{

    @Test
    @Category(CriticalTest.class)
    public void testTestController() {
        commonAssertions(new TestController());
    }
    @Test
    @Category(RegularTest.class)
    public void testRegular() {

    }

    @Test
    @Category(RegularTest.class)
    public void testCodexValueOf() {
        TestController t = new TestController();
        randomizeAllInputs();
        System.out.println(db.driverinput.toString());
    }

}
