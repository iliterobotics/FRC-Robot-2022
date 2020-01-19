package us.ilite.robot.modules;

import us.ilite.common.types.EMatchMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

import static us.ilite.common.types.input.ELogitech310.*;
import us.ilite.robot.Robot;
import static us.ilite.common.types.EFlywheelData.*;

public class FlywheelPrototype extends Module{
    private final TalonFX master = new TalonFX(50);
    private final TalonFX follower = new TalonFX(51);

    public FlywheelPrototype() {

        follower.follow(master);
        follower.setInverted(TalonFXInvertType.FollowMaster);

        System.err.println("CREATED TALON FX's");
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {


    }

    @Override
    public void readInputs(double pNow) {
        Robot.mData.flywheel.set(CURRENT_FLYWHEEL_VELOCITY, 0d);
    }

    @Override
    public void setOutputs(double pNow) {
        master.set(
                ControlMode.PercentOutput,
                Robot.mData.driverinput.get(LEFT_Y_AXIS)
        );
        System.out.println("FLYWHEEL %: " + Robot.mData.driverinput.get(LEFT_Y_AXIS));
    }

    @Override
    public void shutdown(double pNow) {

    }
}
