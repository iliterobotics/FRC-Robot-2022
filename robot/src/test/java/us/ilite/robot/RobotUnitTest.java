package us.ilite.robot;

import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.lang.EnumUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.Assert;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.RegularTest;
import us.ilite.common.Data;
import us.ilite.common.types.input.ELogitech310;
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
    @Category(RegularTest.class)
    public void testUnusedCodex() {
        TestController t = TestController.getInstance();
        randomizeAllInputs();
        t.update();
        System.out.println(t.getUnusedCodexReport());
    }

    @Test
    @Category(CriticalTest.class)
    public void testCodexSet() {
        db.driverinput.set(ELogitech310.B_BTN, Data.NULL_CODEX_VALUE);
        Assert.assertTrue("B_BTN should be null: ", db.driverinput.isNull(ELogitech310.B_BTN));
        db.driverinput.reset();
        Assert.assertTrue("B_BTN should be null after reset: ", db.driverinput.isNull(ELogitech310.B_BTN));
        db.driverinput.set(ELogitech310.A_BTN, 1.0);
        Assert.assertTrue("A_BTN should be set: ", db.driverinput.isSet(ELogitech310.A_BTN));
        db.driverinput.set(ELogitech310.LEFT_X_AXIS, 0.5);
        Assert.assertTrue("Left X Axis should have a value: ", db.driverinput.isSet(ELogitech310.LEFT_X_AXIS));
        Assert.assertTrue("Left Y Axis should NOT have a value: ", db.driverinput.isNull(ELogitech310.LEFT_Y_AXIS));
        db.driverinput.reset();
        Assert.assertTrue("B_BTN should be null after reset: ", db.driverinput.isNull(ELogitech310.B_BTN));
        Assert.assertTrue("Left X Axis should NOT have a value after reset: ", db.driverinput.isNull(ELogitech310.LEFT_X_AXIS));
        Assert.assertTrue("Left Y Axis should NOT have a value after reset: ", db.driverinput.isNull(ELogitech310.LEFT_Y_AXIS));
    }
}
