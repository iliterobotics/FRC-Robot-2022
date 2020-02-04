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
        System.out.println("Adding tab & #");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.addConnectionListener(ev -> System.out.println("CONNECTED"), true);
        inst.startClient("localhost");
        NetworkTable table = inst.getTable("datatable");
        NetworkTableEntry e = table.getEntry("TESTING A THING");
        e.setDouble(200d);
        Shuffleboard.getTab("TEST").addNumber("TESTING A THING", () -> 1000d);
//        e.setDouble(100d);
        SmartDashboard.putNumber("TESTING A THING", 100d);
        Shuffleboard.update();
    }

}
