package us.ilite.robot.modules;

import com.revrobotics.ColorMatchResult;
import org.junit.Test;
import us.ilite.robot.utils.ColorUtils;

import static org.junit.Assert.*;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorTest {

    @Test
    public void test_getColorStringForMatchResult_null() {
        String returnString = ColorSensor.getColorStringForMatchResult(null);

        assertNotNull(returnString);
        assertEquals("NULL MATCH", returnString);
    }

    @Test
    public void test_getColorStringForMatchResult_null_color() {
        ColorMatchResult fakeMatchResult = new ColorMatchResult(null, 1);

        String returnString = ColorSensor.getColorStringForMatchResult(fakeMatchResult);
        assertNotNull(returnString);
        assertEquals("Unknown: null", returnString);
    }

    @Test
    public void test_getColorStringForMatchResult_match_Blue() {
        testColor(ColorUtils.kBlueTarget, "Blue");
    }
    @Test
    public void test_getColorStringForMatchResult_match_Yellow() {
        testColor(ColorUtils.kYellowTarget, "Yellow");
    }
    @Test
    public void test_getColorStringForMatchResult_match_Green() {
        testColor(ColorUtils.kGreenTarget, "Green");
    }
    @Test
    public void test_getColorStringForMatchResult_match_Red() {
        testColor(ColorUtils.kRedTarget, "Red");
    }

    @Test
    public void test_randomColor() {
        testColor(Color.kMagenta, "Unknown: RGB={1.0, 0.0, 1.0}");
    }

    private void testColor(Color color, String expectedString) {
        ColorMatchResult fakeMatchResult = new ColorMatchResult(color, 1);

        String returnString = ColorSensor.getColorStringForMatchResult(fakeMatchResult);
        assertNotNull(returnString);
        assertEquals(expectedString, returnString);
    }
}
