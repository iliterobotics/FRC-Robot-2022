package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.paths.*;
import us.ilite.robot.BaseTest;
import org.reflections.Reflections;

import static us.ilite.robot.auto.paths.BobUtils.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.*;
import java.util.stream.Collectors;

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

    private static final Set<Class<? extends Path>> PATH_CLASSES = Collections.unmodifiableSet(new HashSet<>(Arrays.asList(
            Loop.class,
            OurTrench.class,
            Squiggle.class,
            UNIT_TEST_STRAIGHT_LINE.class,
            Wonky.class,
            Yoink.class
    )));

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

    /**
     * Method to test the method {@link BobUtils#getAvailablePathClasses(Reflections)}
     * with a mocked {@link Reflections}. This test will ensure that a reflections that
     * returns an empty set causes the method being tested to return an empty set.
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePathClasses() {
        Reflections reflection = mock(Reflections.class);
        Set<Class<? extends Path>> availablePathClasses = getAvailablePathClasses(reflection);
        assertNotNull(availablePathClasses);
        assertTrue(availablePathClasses.isEmpty());
    }

    /**
     * Method to test the method {@link BobUtils#getAvailablePathClasses(Reflections)}
     * with null {@link Reflections}. When passed a null, the method should return an
     * empty set
     */
    @Test
    @Category(CriticalTest.class)
    public void test_getAvailablePathClasses_null() {
        Reflections reflection = null;
        Set<Class<? extends Path>> availablePathClasses = getAvailablePathClasses(reflection);
        assertNotNull(availablePathClasses);
        assertTrue(availablePathClasses.isEmpty());
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

        assertEquals(PATH_CLASSES.size(), availablePathClasses.size());
        availablePathClasses.retainAll(PATH_CLASSES);
        assertEquals(PATH_CLASSES.size(), availablePathClasses.size());
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

        assertEquals(PATH_CLASSES.size(), loadedClasses.size());
        loadedClasses.retainAll(PATH_CLASSES);
        assertEquals(PATH_CLASSES.size(), loadedClasses.size());
    }


}
