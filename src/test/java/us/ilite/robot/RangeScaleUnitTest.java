package us.ilite.robot;

import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.RegularTest;
import us.ilite.common.lib.util.RangeScale;

public class RangeScaleUnitTest {


    @Test
    @Category(RegularTest.class)
    public void rangeScaleTest() {
        RangeScale rScale = new RangeScale(-1, 1, 0, 135);
        double[] inputA = {-2.0, -1.0, 0.0, 1.0, 2.0};

        for (double i :inputA) {
            System.out.println("Test a to b input = " + i + " scaled output = " + rScale.scaleAtoB(i));
        }

        double[] inputB = {-45.0, 0.0, 45.0, 67.5, 90.0, 135.0, 180.0};

        for (double i :inputB) {
            System.out.println("Test b to a input = " + i + " scaled output = " + rScale.scaleBtoA(i));
        }

        // Now test a reversed range
        RangeScale rScaleRev = new RangeScale(1, -1, 0, 135);

        for (double i :inputA) {
            System.out.println("Test reversed a to b input = " + i + " scaled output = " + rScaleRev.scaleAtoB(i));
        }

        for (double i :inputB) {
            System.out.println("Test b to reversed a input = " + i + " scaled output = " + rScaleRev.scaleBtoA(i));
        }
    }
}
