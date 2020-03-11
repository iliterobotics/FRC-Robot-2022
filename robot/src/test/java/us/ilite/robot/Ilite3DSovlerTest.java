package us.ilite.robot;

import static org.junit.Assert.*;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.junit.Assert;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.Field2020;
import us.ilite.common.lib.util.Utils;
import us.ilite.robot.vision.CameraConfig;
import us.ilite.robot.vision.Ilite3DSolver;
import static java.lang.Math.*;

import java.util.List;

public class Ilite3DSovlerTest extends BaseTest{

    private static final CameraConfig config = CameraConfig.LIMELIGHT_V2_LOW_RES.setLensHeight(41).setElevationAngle(45);
    private static final Ilite3DSolver flatsolver = new Ilite3DSolver(config, Distance.fromInches(98), Distance.fromInches(34));
    private static final Ilite3DSolver offsetsolver = new Ilite3DSolver(config, Distance.fromInches(98), Distance.fromInches(34), Distance.fromInches(29.25));

    @Test
    @Category(CriticalTest.class)
    public void testOffset() {
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

    private static final double epsilon = 1e-6;

    @Test
    @Category(CriticalTest.class)
    public void testFlatSolver() {
        double hres = config.hResolution;
        double vres = config.vResolution;
//        flatSolve(1.0*h/5.0, 1.0*v/3.0, 3.0*h/5.0, 3.0*v/6.0);

        // 2 corners are the same distance from the center and both have the same y value, then the azimuth should be 0
        String result;
        result = solve(flatsolver, 0.25*hres, 0.50*vres, 0.75*hres, 0.5*vres);
        Assert.assertTrue(result + " ==> should have a = 0", Math.abs(flatsolver.azimuth().degrees()) <= epsilon);
        // Off-center to the right, but still level
//        for(double d = 0; d <= 0.5; d+= 0.05) {
//            result = solve(flatsolver, d*h, 0.50*v, (d+0.5)*h, 0.5*v);
//            System.out.println(result);
//        }
//        for(double d = 0; d <= 0.5; d+= 0.05) {
//            result = solve(offsetsolver, d*h, 0.50*v, (d+0.5)*h, 0.5*v);
//            System.out.println(result);
//        }

        for(int i = 0; i < 200; i++) {
            double w = random() * 0.45 + 0.05;
            double x1 = random() * (1-w);
            double x2 = (x1+w);
            double ht = random() * 0.15;
            boolean flip = random() > 0.5;
            double y1 = max(random()*0.5 + 0.5 - 2*ht,0);
            double y2 = y1+ht;
            double angle = random() * 45.0 + 23.0;
            config.setElevationAngle(angle);
            result = solve(offsetsolver, x1*hres, (flip ?y2:y1)*vres, x2*hres, (flip?y1:y2)*vres);
//            System.out.println(String.format("{%f : (%f,%f),(%f,%f)-->%b}", angle,x1*hres,y1*vres,x2*hres,y2*vres,flip));
            if(abs(offsetsolver.azimuth().degrees()) < 1) {
                System.out.println(result);
            }
        }
    }

    private String solve(Ilite3DSolver s, double x1, double y1, double x2, double y2) {
        s.updatePoseToGoal(List.of(new Translation2d(x1, y1), new Translation2d(x2, y2)));
        return printSolver("Flat", s);
    }

    private String printSolver(String pName, Ilite3DSolver s) {
        StringBuilder sb = new StringBuilder();
        sb.append("L:").append(inches(s.leftRange()));
        sb.append("\tR:").append(inches(s.rightRange()));
        sb.append("\tr:").append(inches(s.range()));
        if(s.offsetAzimuth() != null) {
            sb.append("\te:").append(degrees(s.offsetAzimuth()));
        }
        sb.append("\ta:").append(degrees(s.azimuth()));
        sb.append("\tx:").append(inches(s.x()));
        sb.append("\ty:").append(inches(s.y()));

        return sb.toString();
    }

    private String inches(Distance d) {
        return nf.format(d.inches());
    }

    private String degrees(Angle a) {
        return nf.format(a.degrees());
    }

}
