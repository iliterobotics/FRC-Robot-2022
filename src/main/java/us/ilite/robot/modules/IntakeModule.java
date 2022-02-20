package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;

public class IntakeModule extends Module{
    //Motors
    private TalonFX mIntakeRoller;
    //Solenoids
    private DoubleSolenoid mArmSolenoid;
    //PID Controller and Gains
    private PIDController mRollerPID;
    private ProfileGains kIntakeGains = new ProfileGains().p(0.000001).i(0).d(0);

    private Compressor mCompressor;
    // INTAKE GEAR RATIOS AND CONVERSIONS
    // DO NOT MODIFY THESE PLEASE
    private static final double kMaxFalconSpeed = 6380;
    public static final double kIntakeRollerRatio = 1 / 4;
    public static final double kWheelDiameter = 2.0 / 12.0;
    public static final double kScaledUnitsToRPM = (600 / 2048) * kIntakeRollerRatio;
    public static final double kFeetSpeedConversion = kScaledUnitsToRPM * kWheelDiameter * Math.PI / 60.0;

    public IntakeModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kINRoller);
        mIntakeRoller.setInverted(true);
        mArmSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, Settings.HW.PCH.kINPNIntakeForward, Settings.HW.PCH.kINPNIntakeReverse);
        mRollerPID = new PIDController(kIntakeGains, -kMaxFalconSpeed * kFeetSpeedConversion, kMaxFalconSpeed * kFeetSpeedConversion, 0.1);
        mRollerPID.setOutputRange(-kMaxFalconSpeed * kFeetSpeedConversion, kMaxFalconSpeed * kFeetSpeedConversion);
        mCompressor = new Compressor(20, PneumaticsModuleType.REVPH);
        mCompressor.enableAnalog(100, 110);
    }

    @Override
    public void readInputs() {
        db.cargo.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kFeetSpeedConversion);
        db.cargo.set(CURRENT_ROLLER_RPM, mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        //TODO figure out difference between stator and supply
        db.cargo.set(INTAKE_SUPPLY_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.cargo.set(INTAKE_STATOR_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.cargo.set(RETRACT, mArmSolenoid.get());
        db.cargo.set(EXTEND, mArmSolenoid.get());
        SmartDashboard.putNumber("Compressor PSI", mCompressor.getPressure());
    }

    @Override
    public void setOutputs() {
       setPneumaticIntake();
       setRollerState();
    }

    //if pneumatic state is 1/on, set solenoids on
    public void setPneumaticIntake() {
        Enums.EArmState mode = db.cargo.get(ARM_STATE, Enums.EArmState.class);
        if (mode == null) {
           return;
        }
        switch (mode) {
            case DEFAULT:
                mArmSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            case RETRACT:
                mArmSolenoid.set(DoubleSolenoid.Value.kForward);
                break;
        }
    }

    public void setRollerState() {
        Enums.EIntakeState mode = db.cargo.get(STATE, Enums.EIntakeState.class);
        if (mode == null) {
            mode = Enums.EIntakeState.PERCENT_OUTPUT;
        }
        switch (mode) {
            case PERCENT_OUTPUT:
                mIntakeRoller.set(TalonFXControlMode.PercentOutput, db.cargo.get(DESIRED_PCT));
                break;
            case VELOCITY:
                mRollerPID.setSetpoint(db.cargo.get(SET_ROLLER_VEL_ft_s));
                double speed = mRollerPID.calculate(db.cargo.get(ROLLER_VEL_ft_s), db.cargo.get(SET_ROLLER_VEL_ft_s));
                mIntakeRoller.set(TalonFXControlMode.Velocity, speed);
                break;
        }

    }
}