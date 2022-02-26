package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.robot.hardware.SparkMaxFactory;

public class ClimberModule extends Module{

    private TalonFX mClimb1;
    private TalonFX mClimb2;

    public ClimberModule() {
        mClimb1 = new TalonFX(Settings.HW.CAN.kCLM1);
        mClimb2 = new TalonFX(Settings.HW.CAN.kCL2);
        mClimb2.follow(mClimb1);
    }
    @Override
    public void readInputs() {
    }

    @Override
    public void setOutputs() {
        mClimb1.set(TalonFXControlMode.PercentOutput, 0.1);
    }
}
