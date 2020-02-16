package us.ilite.robot.auto.paths;

import com.fasterxml.jackson.databind.ser.Serializers;
import com.flybotix.hfr.codex.CodexOf;
import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.reflections.Reflections;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EAutonSelectionData;
import us.ilite.robot.Robot;
import us.ilite.robot.controller.AutonCalibration;
import us.ilite.robot.controller.BaseAutonController;

import java.lang.reflect.InvocationTargetException;
import java.util.*;

public class AutonSelection {
    private static Map<String, Path> kAllPaths = BobUtils.getAvailablePaths();
    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");
    private int mPathNumber = mAutonConfiguration.add("Path Number", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 9, "block increment", 1))
            .getEntry()
            .getNumber(0)
            .intValue();
    public static double mDelaySeconds = mAutonConfiguration.add("Path Delay Seconds", 0)
            .getEntry()
            .getDouble(0.0);

    public static Integer mControllerNumber = mAutonConfiguration.add("Controller Number", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", getAutonControllers().size(), "block increment", 1))
            .withSize(4, 1)
            .withPosition(0, 1)
            .getEntry().getNumber(0).intValue();

    public static BaseAutonController mSelectedAutonController;
    public static double mDelayCycleCount = mDelaySeconds / 0.02;
    static {
        int pathIndex = 0;
        mAutonConfiguration.addPersistent("Path Selection", "Select paths by clicking on the 'Path Number' slider dot and using arrow keys").withPosition(0, 1).withSize(4, 1);
        for (Map.Entry<String, Path> entry : kAllPaths.entrySet()) {
            mAutonConfiguration.addPersistent(entry.getKey(), pathIndex).withSize(1, 1).withPosition(pathIndex, 2);
            pathIndex++;
        }
    }

    public AutonSelection() {
        int displayIndex = 0;
        // TODO - do this after selection from shuffleboard
        //  setActivePath(kAllPaths.get((String) kAllPaths.keySet().toArray()[mPathNumber]));
        for (Map.Entry<String, BaseAutonController> entry : getAutonControllers().entrySet()) {
            System.out.println("Adding " + entry.getKey());
            mAutonConfiguration.addPersistent(String.format("%s", displayIndex), entry.getKey())
            .withPosition(displayIndex, 0);
            displayIndex ++;
        }
//        mSelectedAutonController = getAutonControllers().get((String) getAutonControllers().keySet().toArray()[mControllerNumber]);
//        Robot.DATA.autonSelection.set(EAutonSelectionData.DELAY_COUNT_CYCLES, mDelayCycleCount);
    }

//    private static Set<Class<? extends BaseAutonController>> getControllers() {
//        Reflections reflections = new Reflections(Settings.CONTROLLER_PATH_PACKAGE);
//        return getControllers(reflections);
//    }
//
//    private static Set<Class<? extends BaseAutonController>> getControllers(Reflections reflections) {
//        if (reflections == null) {
//            return Collections.emptySet();
//        }
//        return reflections.getSubTypesOf(BaseAutonController.class);
//    }
//
//    private static List<BaseAutonController> getAutonControllers() {
//        ArrayList<BaseAutonController> mControllers = new ArrayList<>();
//        for (Class<?> c : getControllers()) {
//            try {
//
//            }
//        }
//    }

    public static Map<String, BaseAutonController> getAutonControllers() {
        Map<String, BaseAutonController> mControllers = new HashMap<>();
        mControllers.put("AutonCalibration", new AutonCalibration());
        return mControllers;
    }
}
