package us.ilite.robot.modules;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.lang.EnumUtils;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;

import java.util.List;

public class OperatorInput extends Module {
    protected static final double
            DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(OperatorInput.class);

    private Joystick mDriverJoystick;
    private Joystick mOperatorJoystick;
    private EMatchMode mMode = EMatchMode.DISABLED;

    public OperatorInput() {
        mDriverJoystick = new Joystick(0);
        mOperatorJoystick = new Joystick(1);
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        mMode = pMode;
    }

    @Override
    public void readInputs(double pNow) {
        ELogitech310.map(Robot.DATA.driverinput, mDriverJoystick);
        ELogitech310.map(Robot.DATA.operatorinput, mOperatorJoystick);
    }

    @Override
    public void setOutputs(double pNow) {
        if (mMode == EMatchMode.TEST) {
            for(RobotCodex c : Robot.DATA.mLoggedCodexes) {
                String codex = c.meta().getEnum().getSimpleName();
                List<Enum<?>> enums = EnumUtils.getEnums(c.meta().getEnum(), true);
                for(int i = 0; i < enums.size(); i++) {
                    codex += ":" + enums.get(i).name();
                    SmartDashboard.putNumber(codex, (double)c.get(i));
                }
            }
        }
    }

}
