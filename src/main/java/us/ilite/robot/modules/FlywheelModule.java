package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.EMatchMode;
import static us.ilite.robot.Enums.*;

import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EShooterSystemData.*;


public class FlywheelModule extends Module {


    @Override
    public void readInputs() {

    }

    @Override
    public void setOutputs() {

    }
}