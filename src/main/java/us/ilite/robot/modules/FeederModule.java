package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EVisionGoal2020;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.EIntakeData.ROLLER_VEL_ft_s;
import static us.ilite.common.types.EIntakeData.SET_ROLLER_VEL_ft_s;
import static us.ilite.robot.Enums.*;

import us.ilite.robot.hardware.ContinuousRotationServo;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EFeederData.*;


public class FeederModule extends Module {

    //Motors
    private TalonSRX mIntakeFeeder;

    //Beam Breakers
    private DigitalBeamSensor mEntryBeamBreaker;
    private DigitalBeamSensor mExitBeamBreaker;

    //PID Controller and Gains
    private PIDController mFeederPID;
    private ProfileGains kFeederGains = new ProfileGains().p(0.5).i(0).d(0);

    //Constants
    private final double kWheelCircumference = 4 * Math.PI;
    private final double kDebounceTime = 0.1;
    private final double kVelocityConversion = 2048 * 1000 * kWheelCircumference;
    private final double kMaxFalconSpeed = 6380;

    public FeederModule () {
        mIntakeFeeder = new TalonSRX(11);
        mEntryBeamBreaker = new DigitalBeamSensor(9, kDebounceTime);
        mExitBeamBreaker = new DigitalBeamSensor(7, kDebounceTime);
        mFeederPID = new PIDController(kFeederGains, -kMaxFalconSpeed, kMaxFalconSpeed, 0.02);
    }

    @Override
    public void modeInit(EMatchMode mode) {
        db.feeder.set(NUM_BALLS, 0);
    }

    @Override
    public void readInputs() {
        db.feeder.set(CONVEYOR_pct, mIntakeFeeder.getSelectedSensorVelocity()* kVelocityConversion / kMaxFalconSpeed);
        db.feeder.set(ENTRY_BEAM, mEntryBeamBreaker.isBroken());
        db.feeder.set(EXIT_BEAM, mExitBeamBreaker.isBroken());

    }

    @Override
    public void setOutputs() {
        //calculate pid velocity
        mFeederPID.setSetpoint(0.3);
        double desiredVelocity = mFeederPID.calculate(0, clock.getCurrentTimeInMillis());

        mIntakeFeeder.set(TalonSRXControlMode.PercentOutput, desiredVelocity);

        SmartDashboard.putNumber("PID Calc: ", desiredVelocity);
        SmartDashboard.putNumber("Actual VELOCIty: ", db.feeder.get(CONVEYOR_pct));
        SmartDashboard.putNumber("Desired VELOCIty: ", db.feeder.get(SET_CONVEYOR_pct));
//        SmartDashboard.putNumber("numballs: ", db.feeder.get(NUM_BALLS));
    }
}