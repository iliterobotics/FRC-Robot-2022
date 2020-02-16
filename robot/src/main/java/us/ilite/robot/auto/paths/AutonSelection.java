package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.reflections.Reflections;
import us.ilite.common.config.Settings;
import us.ilite.robot.controller.BaseAutonController;

import java.util.*;

public class AutonSelection {
    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");
    private static double mDelaySeconds = mAutonConfiguration.add("Path Delay Seconds", 0)
            .getEntry()
            .getDouble(0.0);

    private static int mControllerNumber = mAutonConfiguration.add("Controller Number", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", getAutonControllers().size(), "block increment", 1))
            .withSize(4, 1)
            .withPosition(0, 1)
            .getEntry().getNumber(0).intValue();

    public BaseAutonController mSelectedAutonController = getAutonControllers().get(mControllerNumber);
    public double mDelayCycleCount = mDelaySeconds / 0.02;

    public AutonSelection() {
        int displayIndex = 0;
        for (Class c : getControllers()) {
            mAutonConfiguration.addPersistent(String.format("%s", displayIndex), c.getSimpleName());
            displayIndex ++;
        }

    }

    private static Set<Class<? extends BaseAutonController>> getControllers(Reflections reflections) {
        if(reflections == null) return Collections.emptySet();
        return reflections.getSubTypesOf(BaseAutonController.class);
    }

    private static Set<Class<? extends BaseAutonController>> getControllers() {
        Reflections reflections = new Reflections(Settings.CONTROLLER_PATH_PACKAGE);
        return getControllers(reflections);
    }

    private static List<BaseAutonController> getAutonControllers() {
        ArrayList<BaseAutonController> allControllers = new ArrayList<>();
        BaseAutonController[] controllers = (BaseAutonController[]) getControllers().toArray();
        Collections.addAll(allControllers, controllers);
        return allControllers;
    }
}
