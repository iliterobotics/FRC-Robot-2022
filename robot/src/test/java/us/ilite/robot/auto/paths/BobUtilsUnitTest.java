package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.junit.Test;
import us.ilite.paths.Squiggle;
import us.ilite.paths.UNIT_TEST_STRAIGHT_LINE;
import us.ilite.paths.Wonky;
import us.ilite.paths.Yoink;
import us.ilite.robot.BaseTest;

import static us.ilite.robot.auto.paths.BobUtils.*;
import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.List;

public class BobUtilsUnitTest extends BaseTest {
    private static final Path UNIT_TEST_STRAIGHT_LINE = new UNIT_TEST_STRAIGHT_LINE();
    private static final Path SQUIGGLE = new Squiggle();
    private static final Path WONKY = new Wonky();
    private static final Path YOINK = new Yoink();
    private static final Path[] ANALYSIS_PATHS = new Path[] {
        UNIT_TEST_STRAIGHT_LINE,
        SQUIGGLE,
        WONKY,
        YOINK
    };

    @Test
    public void testTotalTime() {
        System.out.println("=== TRAJECTORY ANALYSIS ===");
        for (Path p : ANALYSIS_PATHS) {
            StringBuilder sb = new StringBuilder(p.getClass().getSimpleName()).append("\t");
            sb.append("SEGMENTS: " + p.getSegmentCount());
            sb.append("\t");
            sb.append("TIME (s) = " + BobUtils.getPathTotalTime(p));
            sb.append("\t");
            sb.append("MAX VEL (ft/s) = " + BobUtils.getPathMaxVelocity(p));
            sb.append("\t");
            sb.append("INDEX @ 1s = " + getIndexForCumulativeTime(p, 1.0, 0.0));
            sb.append("\t");
            sb.append("INDEX @ 5s = " + getIndexForCumulativeTime(p, 5.0, 0.0));
            System.out.println(sb);
        }

    }

    @Test
    public void testTrajectoryMapping() {
        System.out.println("=== TRAJECTORY MAPPING ===");
        Trajectory.State goal = sample(UNIT_TEST_STRAIGHT_LINE, 1.0, 0.0);
        System.out.println(goal);
    }

    @Test
    public void testCurvature() {
        Path p = WONKY;

        for(int i = 2; i < 102; i+=10) {
            double c = calculateCurvature(p, i);
            System.out.println("Curvature @ " + i * 0.020d + " = " + c);
        }

    }

    @Test
    public void test_getAvailablePathClasses() {
        List<Class<Path>> availablePathClasses = getAvailablePathClasses();
        assertNotNull(availablePathClasses);
        assertFalse(availablePathClasses.isEmpty());
    }
}
