package us.ilite.robot;

import static org.junit.Assert.*;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.Field2020;
import us.ilite.common.lib.util.Utils;
import us.ilite.robot.vision.CameraConfig;
import us.ilite.robot.vision.Ilite3DSolver;

import java.util.List;

public class Ilite3DSovlerTest extends BaseTest{

    private static final CameraConfig config = CameraConfig.LIMELIGHT_V2_LOW_RES.setLensHeight(45).setElevationAngle(45);
    private static final Ilite3DSolver flatsolver = new Ilite3DSolver(config, Distance.fromInches(30), Distance.fromInches(34));
    private static final Ilite3DSolver offsetsolver = new Ilite3DSolver(config, Distance.fromInches(30), Distance.fromInches(34), Distance.fromInches(29.25));

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

    @Test
    @Category(CriticalTest.class)
    public void testFlatSolver() {
        // Perfected centered
        Translation2d left = new Translation2d(1.0*config.hResolution/4.0, config.vResolution/2.0);
        Translation2d right = new Translation2d(3.0*config.hResolution/4.0, config.vResolution/2.0);
        flatsolver.updatePoseToGoal(List.of(left, right));
        System.out.println(printSolver("Flat", flatsolver));
    }

    private String printSolver(String pName, Ilite3DSolver pSolver) {
        StringBuilder sb = new StringBuilder(pName + ": ");
        sb.append("\tx-").append(inches(pSolver.x()));
        sb.append("\ty-").append(inches(pSolver.y()));
        sb.append("\tr-").append(inches(pSolver.range()));
        sb.append("\ta-").append(degrees(pSolver.azimuth()));
        if(pSolver.offsetAzimuth() != null) {
            sb.append("\te-").append(degrees(pSolver.offsetAzimuth()));
        }
        sb.append("\tL-").append(inches(pSolver.leftRange()));
        sb.append("\tR-").append(inches(pSolver.rightRange()));

        return sb.toString();
    }

    private String inches(Distance d) {
        return nf.format(d.inches());
    }

    private String degrees(Angle a) {
        return nf.format(a.degrees());
    }

}
