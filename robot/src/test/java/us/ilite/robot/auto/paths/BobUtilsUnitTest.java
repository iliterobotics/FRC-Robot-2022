package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.junit.Test;
import java.lang.Math.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static us.ilite.robot.auto.paths.BobUtils.*;

public class BobUtilsUnitTest {
    private static final Path
            T_LINE_27_FT = new T_LINE_27_FT(),
            T_90DEG_12FT = new T_90DEG_12FT(),
            T_180DEG_24FT = new T_180DEG_24FT(),
            SQUIGGLE = new Squiggle(),
            WONKY = new Wonky(),
            YOINK = new Yoink(),
            LOOP = new Loop(),
            OURTRENCH = new OurTrench(),
            TRENCH5 = new Trench5()
    ;
    private static final Path[] ANALYSIS_PATHS = new Path[] {
            SQUIGGLE,
            WONKY,
            LOOP,
            OURTRENCH,
            TRENCH5,
            YOINK,
    };

    private static final Path[] TEST_PATHS = new Path[] {
            T_LINE_27_FT,
            T_90DEG_12FT,
            T_180DEG_24FT
    };

    private static final Path[] ALL_PATHS = new Path[] {
            SQUIGGLE,
            WONKY,
            LOOP,
            OURTRENCH,
            TRENCH5,
            YOINK,
            T_LINE_27_FT,
            T_90DEG_12FT,
            T_180DEG_24FT
    };

    @Test
    public void testTotalTime() {
        System.out.println("=== TRAJECTORY ANALYSIS ===");
        for (Path p : ALL_PATHS) {
            StringBuilder sb = new StringBuilder(p.getClass().getSimpleName()).append("\t");
            append("SEGS", p.getSegmentCount(), sb);
            append("DT (s)", getPathTotalTime(p), sb);
            append("DIST (FT)", getPathTotalDistanceFt(p), sb);
            append("MAX VEL (ft/s)", getPathMaxVelocity(p), sb);
            append("AVG VEL (ft/s)", getPathTotalDistanceFt(p) / getPathTotalTime(p), sb);
            append("INDEX @ 1s", getIndexForCumulativeTime(p, 1.0, 0.0), sb);
            append("INDEX @ 5s", getIndexForCumulativeTime(p, 5.0, 0.0), sb);
            System.out.println(sb);
        }

    }

    private static void append(String pLabel, double pNumber, StringBuilder sb) {
        sb.append(pLabel + " = ").append(nf.format(pNumber)).append("\t");
    }

    private static void csv(StringBuilder sb, double pNumber) {
        sb.append(csvf.format(pNumber)).append(",");
    }

    @Test
    public void testTrajectoryMapping() {
        System.out.println("=== TRAJECTORY MAPPING ===");
        Trajectory.State goal = sample(T_LINE_27_FT, 1.0, 0.0);
        System.out.println(goal);
    }

    @Test
    public void testCurvature() {
        System.out.println("=== CURVATURE === (deg / yd)");
        for(Path p : ALL_PATHS) {
            StringBuilder sb = new StringBuilder(p.getClass().getSimpleName()).append("\t");
            for(int i = 2; i < p.getSegmentCount(); i += 5) {
                csv(sb, 180/Math.PI*calculateCurvature(p, i) * METERS_TO_FEET * 3d);
            }
            System.out.println(sb);
        }
    }

    private static final NumberFormat nf = new DecimalFormat("0.00");
    private static final NumberFormat csvf = new DecimalFormat("0.0000");
}
