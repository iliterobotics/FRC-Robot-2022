package us.ilite.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import us.ilite.robot.controller.*;

import java.lang.reflect.InvocationTargetException;

public class AutonSelection {
    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Configuration");
    public static int mDelaySeconds;
    private SendableChooser<Class<?>> mSendableAutonControllers = new SendableChooser<>();

    /**
     * Update these Auton Controllers whenever new ones are added
     */
    private Class<?>[] mAutonControllers = {

    };

    public AutonSelection() {
       mDelaySeconds = ((Double) (mAutonConfiguration.add("Path Delay Seconds", 0)
               .withPosition(2, 0)
               .withSize(2, 1)
               .getEntry()
               .getDouble(0.0)))
               .intValue();

        mSendableAutonControllers.setDefaultOption("Default - Auton Calibration", BaseAutonController.class);
        for (Class<?> c : mAutonControllers) {
            mSendableAutonControllers.addOption(c.getSimpleName(), c);
        }

        mAutonConfiguration.add("Choose Auton Controller", mSendableAutonControllers)
            .withPosition(0, 0)
            .withSize(2, 1);
    }

    public BaseAutonController getSelectedAutonController() {
        try {
            return (BaseAutonController) mSendableAutonControllers.getSelected().getDeclaredConstructor().newInstance();
        } catch (NoSuchMethodException nsme) {
            nsme.printStackTrace();
        } catch (InstantiationException ie) {
            ie.printStackTrace();
        } catch (IllegalAccessException iae) {
            iae.printStackTrace();
        } catch (InvocationTargetException ite) {
            ite.printStackTrace();
        }
        // THIS SHOULD NEVER BE REACHED
        return null;
    }
}
