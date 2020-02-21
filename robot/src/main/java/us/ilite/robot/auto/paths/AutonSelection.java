package us.ilite.robot.auto.paths;

import com.team319.trajectory.Path;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.robot.controller.*;

import java.util.*;

public class AutonSelection {
    private static Map<String, Path> kAllPaths = BobUtils.getAvailablePaths();
    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");

    public static double mDelaySeconds = mAutonConfiguration.add("Path Delay Seconds", 0)
            .withPosition(2, 3)
            .getEntry()
            .getDouble(0.0);

    public static double mDelayCycleCount = mDelaySeconds / 0.02;

    private SendableChooser<BaseAutonController> mSendableAutonControllers = new SendableChooser<>();
    private SendableChooser<Path> mSendablePaths = new SendableChooser<>();
    /**
     * Update these Auton Controllers whenever new ones are added
     */
    private BaseAutonController[] mAutonControllers = {
            new LineAutonController(),
            new ShootIntakeController()
    };

    public AutonSelection() {
        updateAutonControllers();
    }

    public void updateAutonControllers() {
        for (BaseAutonController c : mAutonControllers) {
            mSendableAutonControllers.addOption(c.getClass().getSimpleName(), c);
        }
        mSendableAutonControllers.setDefaultOption("Default - Auton Calibration", new AutonCalibration());
        mAutonConfiguration.add("Choose Auton Controller", mSendableAutonControllers);
    }

    public BaseAutonController getSelectedAutonController() {
        return mSendableAutonControllers.getSelected();
    }
}
