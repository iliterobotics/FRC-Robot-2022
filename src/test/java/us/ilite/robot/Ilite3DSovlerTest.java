package us.ilite.robot;

import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Assert;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.Field2022;
import us.ilite.common.lib.util.Utils;
import us.ilite.robot.vision.CameraConfig;
import us.ilite.robot.vision.Ilite3DSolver;
import static java.lang.Math.*;

import java.util.List;

public class Ilite3DSovlerTest extends BaseTest{

    private static final CameraConfig config = CameraConfig.LIMELIGHT_V2_LOW_RES.setLensHeight(41).setElevationAngle(45);
    private static final Ilite3DSolver solver = new Ilite3DSolver(config, Distance.fromInches(98), Distance.fromInches(34), Distance.fromInches(21));
    private static final double epsilon = 1e-6;

    @Test
    @Category(CriticalTest.class)
    public void testsolver() {
        double hres = config.hResolution;
        double vres = config.vResolution;
//        flatSolve(1.0*h/5.0, 1.0*v/3.0, 3.0*h/5.0, 3.0*v/6.0);

        // 2 corners are the same distance from the center and both have the same y value, then the azimuth should be 0
        String result;
        config.setElevationAngle(45.0);
        result = solve(solver, 0.25*hres, 0.50*vres, 0.75*hres, 0.5*vres);
        System.out.println(result);
        Assert.assertTrue(result + " ==> should have a = 0", Math.abs(solver.azimuth().degrees()) <= epsilon);
        double symmetricrange = solver.range().inches();
        config.setInverted(true);
        result = solve(solver, 0.25*hres, 0.50*vres, 0.75*hres, 0.5*vres);
        System.out.println(result);
        Assert.assertTrue(result + " ==> should have a = 0", Math.abs(solver.azimuth().degrees()) <= epsilon);
        Assert.assertTrue(
                result + " ==> Should match the non-inverted range of " + nf.format(symmetricrange),
                solver.range().inches() - symmetricrange <= epsilon
        );
        config.setElevationAngle(30.0);
        config.setInverted(false);
        result = solve(solver, 0.25*hres, 0.50*vres, 0.75*hres, 0.5*vres);
        symmetricrange = solver.range().inches();
        config.setInverted(true);
        result = solve(solver, 0.25*hres, 0.50*vres, 0.75*hres, 0.5*vres);
        Assert.assertTrue(
                result + " ==> Should match the non-inverted range of " + nf.format(symmetricrange),
                solver.range().inches() - symmetricrange <= epsilon
        );

        // Off-center to the right, but still level
//        for(double d = 0; d <= 0.5; d+= 0.05) {
//            result = solve(solver, d*h, 0.50*v, (d+0.5)*h, 0.5*v);
//            System.out.println(result);
//        }
//        for(double d = 0; d <= 0.5; d+= 0.05) {
//            result = solve(offsetsolver, d*h, 0.50*v, (d+0.5)*h, 0.5*v);
//            System.out.println(result);
//        }

        int count = 0;
        for(int invert = 0; invert < 2; invert++) {
            for(double elevation = 20; elevation <= 70; elevation += 5.0) {
                for(double targetwidth = 2.5*elevation; targetwidth <= 3.5*elevation; targetwidth+=0.25*elevation) {
                    for(double targetheight = 0; targetheight <= 8; targetheight += 2) {
                        count++;
                        double x1 = random() * (config.hResolution - targetwidth);
                        double x2 = x1 + targetwidth;
                        double y1 = random() * (config.vResolution - targetheight);
                        double y2 = y1 + targetheight;
                        if(invert == 1) {
                            x1 = config.hResolution - x1;
                            x2 = config.hResolution - x2;
                            y1 = config.vResolution - y1;
                            y2 = config.vResolution - y2;
                        }
                        boolean flip = random() > 0.5;
                        config.setInverted(invert == 1);
                        config.setElevationAngle(elevation);
                        result = solve(solver, x1, (flip ?y2:y1), x2, (flip?y1:y2));
                        if(abs(solver.range().inches()) < 12*27) {
                            double inneraz = abs(solver.azimuth().degrees()) - abs(solver.offsetAzimuth().degrees());
                            String goal = "OUTER";//Field2022.canHitInnerGoal(Angle.fromDegrees(inneraz), solver.range()) ? "INNER" : "OUTER";
                            System.out.println(String.format("{e:%.0f,w:%.0f,h:%.0f / (%.0f,%.0f),(%.0f,%.0f)-->\t%s ==> %s}", elevation,targetwidth,targetheight,x1,y1,x2,y2,result,goal));
                        }
                    }
                }
            }
        }
        System.out.println(String.format("Ran %d solve tests",count));

//        for(int i = 0; i < 300; i++) {
//            double w = random() * 0.65 + 0.15;
//            double x1 = random() * (1-w);
//            double x2 = (x1+w);
//            double ht = random() * 0.1;
//            boolean flip = random() > 0.5;
//            double y1 = max(random()*0.5 + 0.5 - 2*ht,0);
//            double y2 = y1+ht;
//            double angle = random() * 45.0 + 23.0;
//            config.setElevationAngle(angle);
//            config.setInverted(random() > 0.5);
//            result = solve(solver, x1*hres, (flip ?y2:y1)*vres, x2*hres, (flip?y1:y2)*vres);
//            if(abs(solver.range().inches()) > 120) {
//                System.out.println(String.format("{%f : (%f,%f),(%f,%f)-->%b}", angle,x1*hres,y1*vres,x2*hres,y2*vres,flip));
//                System.out.println(result + "\ti:" + config.isInverted());
//            }
//        }
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
