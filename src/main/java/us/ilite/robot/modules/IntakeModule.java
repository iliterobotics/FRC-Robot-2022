package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EIntakeData;
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

    // INTAKE GEAR RATIOS AND CONVERSIONS
    // DO NOT MODIFY THESE PLEASE
    private static final double kMaxFalconSpeed = 6380;
    public static final double kIntakeRollerRatio = 1 / 4;
    public static final double kWheelDiameter = 2.0 / 12.0;
    public static final double kScaledUnitsToRPM = (600 / 2048) * kIntakeRollerRatio;
    public static final double kFeetSpeedConversion = kScaledUnitsToRPM * kWheelDiameter * Math.PI / 60.0;

    public IntakeModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kINRoller);
        mArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Settings.HW.PCH.kINPNIntakeForward, Settings.HW.PCH.kINPNIntakeReverse);
        mRollerPID = new PIDController(kIntakeGains, -kMaxFalconSpeed * kFeetSpeedConversion, kMaxFalconSpeed * kFeetSpeedConversion, 0.1);
        mRollerPID.setOutputRange(-kMaxFalconSpeed * kFeetSpeedConversion, kMaxFalconSpeed * kFeetSpeedConversion);
    }

    @Override
    public void readInputs() {
        db.cargo.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kFeetSpeedConversion);
        db.cargo.set(CURRENT_ROLLER_RPM, mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        //TODO figure out difference between stator and supply
        db.cargo.set(INTAKE_SUPPLY_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.cargo.set(INTAKE_STATOR_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.cargo.set(FWD_PNEUMATIC_STATE, mArmSolenoid.get());
        db.cargo.set(REV_PNEUMATIC_STATE, mArmSolenoid.get());
    }

    @Override
    public void setOutputs() {
        setPneumaticIntake();
        setRollerState();
    }

    //if pneumatic state is 1/on, set solenoids on
    public void setPneumaticIntake() {
        if(db.cargo.get(FWD_PNEUMATIC_STATE) == 1d) {
            mArmSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else if (db.cargo.get(REV_PNEUMATIC_STATE) == 1d) {
            mArmSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        else {
            mArmSolenoid.set(DoubleSolenoid.Value.kOff);
        }
    }

    public void setRollerState() {
        Enums.EIntakeState mode = db.cargo.get(STATE, Enums.EIntakeState.class);
        if (mode == null) {
            return;
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