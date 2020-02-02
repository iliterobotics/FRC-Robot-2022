package us.ilite.robot;

import com.team319.trajectory.Path;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import org.reflections.Reflections;
import us.ilite.CriticalTest;
import us.ilite.robot.auto.paths.BobUtils;

import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

@Category(CriticalTest.class)
public class AutonControllerTest {

    @Test
    @Category(CriticalTest.class)
    public void getAutonPaths() {
        Map<String, Path> availablePaths = BobUtils.getAvailablePaths();
        for(String path : availablePaths.keySet()) {
            System.out.println("Path " + path + " has runtime of " + BobUtils.getPathTotalTime(availablePaths.get(path)));
        }
    }

}
