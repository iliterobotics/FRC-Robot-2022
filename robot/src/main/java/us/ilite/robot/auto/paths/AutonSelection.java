package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.reflections.Reflections;
import us.ilite.common.config.Settings;
import us.ilite.robot.controller.BaseAutonController;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

public class AutonSelection {
    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");
    private double mDelaySeconds = mAutonConfiguration.add("Path Delay Seconds", 0).getEntry().getDouble(0.0);
    private int mControllerNumber = mAutonConfiguration.add("Controller Number", 0).getEntry().getNumber(0).intValue();

    private BaseAutonController mActiveController;
    private double mDelayCycleCount;

    public AutonSelection() {
        mDelayCycleCount = mDelaySeconds / 0.02;
        mActiveController = getAutonControllers().get(mControllerNumber);
    }

    static Set<Class<? extends Path>> getControllers(Reflections reflections) {
        if(reflections == null) return Collections.emptySet();
        return reflections.getSubTypesOf(Path.class);
    }

    public static Set<Class<? extends Path>> getControllers() {
        Reflections reflections = new Reflections(Settings.CONTROLLER_PATH_PACKAGE);
        return getControllers(reflections);
    }

    public List<BaseAutonController> getAutonControllers() {
        ArrayList<BaseAutonController> allControllers = new ArrayList<>();
        BaseAutonController[] controllers = (BaseAutonController[]) getControllers().toArray();
        Collections.addAll(allControllers, controllers);
        return allControllers;
    }

    public BaseAutonController getAutonController() {
        return mActiveController;
    }

    public double getDelay() {
        return mDelayCycleCount;
    }
}
