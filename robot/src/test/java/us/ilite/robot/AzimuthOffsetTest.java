package us.ilite.robot;

import static org.junit.Assert.*;
import org.junit.Test;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.Field2020;
import us.ilite.common.lib.util.Utils;

public class AzimuthOffsetTest extends BaseTest{

    @Test
    public void test() {
        Distance offset = Distance.fromInches(29.25);
        Distance d = Distance.fromInches(48);
        double[] expected = new double[] { 13.28, 1.89, 3.78,  5.67, 7.55, 9.42, 11.28, 13.12 };
        double[] a = new double[] { 35.427, 5, 10, 15, 20, 25, 30, 35, };
        boolean[] goal = new boolean[] {false, true, true, true, true, true, false, false};
        for(int i =0 ; i < a.length; i++) {
            Angle thetaT = Angle.fromDegrees(a[i]);
            Angle o = Utils.calculateAngleOffsetY(thetaT, d, offset);
            boolean inner = Field2020.canHitInnerGoal(thetaT.subtract(o), d);
            String msg = (inner ? "INNER " : "OUTER ") + "Distance " + nf.format(d.inches()) + " @ " + a[i] + ", Offset = " + nf.format(o.degrees());
            assertEquals("Incorrect Offset Angle: " + msg, nf.format(o.degrees()), nf.format(expected[i]));
            assertEquals("Incorrect Boundary Test: " + msg, inner, goal[i]);
        }
    }

}
