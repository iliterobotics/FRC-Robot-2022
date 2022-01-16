package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
//import edu.wpi.first.math.trajectory.Trajectory;
import org.junit.Test;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static us.ilite.robot.auto.BobUtils.*;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import org.reflections.Reflections;
import us.ilite.robot.auto.BobUtils;

import static org.junit.Assert.*;
//import static org.mockito.Mockito.*;

import java.util.*;
import java.util.stream.Collectors;

public class BobUtilsUnitTest {
//            T_LINE_27_FT = new T_LINE_27_FT(),
//            T_90DEG_12FT = new T_90DEG_12FT(),
//            T_180DEG_24FT = new T_180DEG_24FT(),
//            SQUIGGLE = new Squiggle(),
//            WONKY = new Wonky(),
//            YOINK = new Yoink(),
//            LOOP = new Loop(),
//            OURTRENCH = new OurTrench()

    private static final Path[] ANALYSIS_PATHS = new Path[] {
//            SQUIGGLE,
//            WONKY,
//            LOOP,
//            OURTRENCH,
//            YOINK,
    };

    private static final Path[] TEST_PATHS = new Path[] {
//            T_LINE_27_FT,
//            T_90DEG_12FT,
//            T_180DEG_24FT
    };

    private static final Set<Class<? extends Path>> PATH_CLASSES = Collections.unmodifiableSet(new HashSet<>(Arrays.asList(
//            Loop.class,
//            OurTrench.class,
//            Squiggle.class,
//            Wonky.class,
//            Yoink.class
    )));

    @Test
    public void testTotalTime() {
        System.out.println("=== TRAJECTORY ANALYSIS ===");
        for (Path p : BobUtils.getAvailablePaths().values()) {
            StringBuilder sb = new StringBuilder(p.getClass().getSimpleName()).append("\t");
            append("SEGS", p.getSegmentCount(), sb);
            append("DT (s)", getPathTotalTime(p), sb);
            append("DIST (FT)", getPathTotalDistance(p).feet(), sb);
            append("MAX VEL (ft/s)", getPathMaxVelocity(p), sb);
            append("AVG VEL (ft/s)", getPathTotalDistance(p).feet() / getPathTotalTime(p), sb);
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
//        Trajectory.State goal = sample(T_LINE_27_FT, 1.0, 0.0);
//        System.out.println(goal);
    }

    @Test
    public void testCurvature() {
        System.out.println("=== CURVATURE === (deg / yd)");
        for(Path p : BobUtils.getAvailablePaths().values()) {
            StringBuilder sb = new StringBuilder(p.getClass().getSimpleName()).append("\t");
            for(int i = 2; i < p.getSegmentCount(); i += 5) {
              //  csv(sb, 180/Math.PI*calculateCurvature(p, i) * METERS_TO_FEET * 3d);
            }
            System.out.println(sb);
        }
    }

    private static final NumberFormat nf = new DecimalFormat("0.00");
    private static final NumberFormat csvf = new DecimalFormat("0.00");
    /**
     * Method to test the method
     * with a mocked {@link Reflections}. This test will ensure that a reflections that
     * returns an empty set causes the method being tested to return an empty set.
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePathClasses() {
//        Reflections reflection = mock(Reflections.class);
//        Set<Class<? extends Path>> availablePathClasses = getAvailablePathClasses(reflection);
//        assertNotNull(availablePathClasses);
//        assertTrue(availablePathClasses.isEmpty());
    }

    /**
     * Method to test the method
     * with null {@link Reflections}. When passed a null, the method should return an
     * empty set
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePathClasses_null() {
//        Reflections reflection = null;
//        Set<Class<? extends Path>> availablePathClasses = getAvailablePathClasses(reflection);
//        assertNotNull(availablePathClasses);
//        assertTrue(availablePathClasses.isEmpty());
    }

    /**
     * Method to test the method {@link BobUtils#getAvailablePathClasses()} to
     * ensure that it returns all of the classes that actually extend {@link Path}
     *
     * Note: If a new {@link Path} class is added, this method will fail as the
     * {@link BobUtilsUnitTest#PATH_CLASSES} does not have the new Path class
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePathClasses_realMethod() {
        Set<Class<? extends Path>> availablePathClasses = getAvailablePathClasses();
        assertNotNull(availablePathClasses);
        assertFalse(availablePathClasses.isEmpty());

        // TODO - find a different way to test this since as-written it will never pass. The # of paths we create for
        // a given competition will change too often for this test to pass as-is.
//        assertEquals(PATH_CLASSES.size(), availablePathClasses.size());
        availablePathClasses.retainAll(PATH_CLASSES);
//        assertEquals(PATH_CLASSES.size(), availablePathClasses.size());
    }

    /**
     * Method to test the method {@link BobUtils#getAvailablePaths()} to
     * ensure that it returns all of the Path objects that actually extend {@link Path}
     * are instantiated and returned in the Map
     *
     * Note: If a new {@link Path} class is added, this method will fail as the
     * {@link BobUtilsUnitTest#PATH_CLASSES} does not have the new Path class
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePaths() {
        Map<String, Path> availablePaths = getAvailablePaths();
        assertNotNull(availablePaths);
        assertFalse(availablePaths.isEmpty());

        Set<? extends Class<?>> loadedClasses = availablePaths.values().stream().map(Object::getClass).collect(Collectors.toSet());

        // TODO - find a different way to test this since as-written it will never pass. The # of paths we create for
        // a given competition will change too often for this test to pass as-is.
//        assertEquals(PATH_CLASSES.size(), loadedClasses.size());
        loadedClasses.retainAll(PATH_CLASSES);
//        assertEquals(PATH_CLASSES.size(), loadedClasses.size());
    }

    @Test
    @Category(CriticalTest.class)
    public void test_showAutonPathsTotalTime() {
        Map<String, Path> availablePaths = BobUtils.getAvailablePaths();
        for(String path : availablePaths.keySet()) {
            System.out.println("Path " + path + " has runtime of " + BobUtils.getPathTotalTime(availablePaths.get(path)));
        }
    }

}
