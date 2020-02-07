package us.ilite.robot;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.lang.EnumUtils;
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

import java.util.List;

@Category(CriticalTest.class)
public class RobotUnitTest extends BaseTest{

    @Test
    @Category(CriticalTest.class)
    public void testTestController() {
        commonAssertions(TestController.getInstance());
    }
    @Test
    @Category(RegularTest.class)
    public void testRegular() {

    }

    @Test
    @Category(CriticalTest.class)
    public void testUnusedCodex() {
        TestController t = TestController.getInstance();
        randomizeAllInputs();
        t.update(0d);
        System.out.println(t.getUnusedCodexReport());
    }

}
