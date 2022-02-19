package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EFeederData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.hardware.DigitalBeamSensor;

import javax.management.relation.Role;

import static us.ilite.common.types.EIntakeData.*;

public class IntakeModule extends Module{

    //Motors
    private TalonFX mIntakeRoller;

    //Solenoids
    private DoubleSolenoid mArmSolenoid;

    //PID Controller and Gains
    private PIDController mRollerPID;
    private ProfileGains kIntakeGains = new ProfileGains().p(0.000001).i(0).d(0);

    //Constants
    private final double kWheelCircumference = 4 * Math.PI;
    private final double kMaxFalconSpeed = 5820; // change back to falcon
    private final double kVelocityConversion = 2048 * 1000 * kWheelCircumference;

    public IntakeModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kMAXIntakeRollerId);
        mArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,-1, -1);
        //initialize motors and such

        //create pid values, set min/max input/outputs
        mRollerPID = new PIDController(kIntakeGains, -kMaxFalconSpeed, kMaxFalconSpeed, 0.1);
        mRollerPID.setOutputRange(-1, 1);
    }

    @Override
    public void readInputs() {
        db.cargo.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kVelocityConversion);
        db.cargo.set(FWD_PNEUMATIC_STATE, mArmSolenoid.get());
        db.cargo.set(REV_PNEUMATIC_STATE, mArmSolenoid.get());
    }

    @Override
    public void setOutputs() {
        //calculate pid velocity
//        mRollerPID.setSetpoint(db.cargo.get(SET_ROLLER_VEL_ft_s));
//        double desiredVelocity = mRollerPID.calculate(db.cargo.get(ROLLER_VEL_ft_s), clock.getCurrentTimeInMillis());

        //set value to motor
        mIntakeRoller.set(TalonFXControlMode.PercentOutput, db.cargo.get(SET_ROLLER_VEL_ft_s) / kMaxFalconSpeed);

        //turning the solenoids on or off
        setPneumaticIntake();
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
}