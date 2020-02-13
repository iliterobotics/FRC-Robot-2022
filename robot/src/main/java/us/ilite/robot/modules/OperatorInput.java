package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;

import java.util.List;
import java.util.Map;

public class OperatorInput extends Module {
    protected static final double
            DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(OperatorInput.class);

    private Joystick mDriverJoystick;
    private Joystick mOperatorJoystick;
    private EMatchMode mMode = EMatchMode.DISABLED;

    private ShuffleboardTab mAutonConfiguration;
    private NetworkTableEntry AutonPathDropdown;
    private NetworkTableEntry mMatchTimeEntry;


    public OperatorInput() {
        mMatchTimeEntry = mAutonConfiguration.add("Match Time", 0).withSize(2, 1).
                withPosition(0, 0).getEntry();
        mDriverJoystick = new Joystick(0);
        AutonPathDropdown = Shuffleboard.getTab("Auton Options").add("Option", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 5))
                .getEntry();

        mOperatorJoystick = new Joystick(1);
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

        if(mDriverJoystick.getType() == null) {
            System.err.println("======= DRIVER JOYSTICK IS NOT PLUGGED IN =======");
        }
        if(mOperatorJoystick.getType() == null) {
            System.err.println("======= OPERATOR JOYSTICK IS NOT PLUGGED IN =======");
        }
        mMode = pMode;
    }

    @Override
    public void readInputs(double pNow) {
        ELogitech310.map(Robot.DATA.driverinput, mDriverJoystick);
        ELogitech310.map(Robot.DATA.operatorinput, mOperatorJoystick);
    }

    @Override
    public void setOutputs(double pNow) {

    }

}
