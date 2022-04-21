package us.ilite.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ClimbModeSelection {
    private SendableChooser<String> mClimbSelection = new SendableChooser<String>();
    private ShuffleboardTab mClimbOptions = Shuffleboard.getTab("Climber Selection");

    public ClimbModeSelection() {
        mClimbOptions.add("Climber Mode", mClimbSelection)
                .withPosition(1, 3)
                .withSize(2, 2);
        mClimbSelection.setDefaultOption("Default", "DCMP");
        mClimbSelection.addOption("District Automation", "DCMP");
        mClimbSelection.addOption("Worlds Automation", "WCMP");
    }
    public String getSelectedMode() {
        return mClimbSelection.getSelected();
    }
}
