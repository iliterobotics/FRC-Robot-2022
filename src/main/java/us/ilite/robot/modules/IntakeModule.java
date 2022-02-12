package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ILITEPIDController;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.hardware.DigitalBeamSensor;

import static us.ilite.common.types.EIntakeData.*;

public class IntakeModule extends Module{

    //Motors
    private TalonFX mIntakeRoller;

    //Solenoids
    private Solenoid mArmSolenoid;

    //PID Controller and Gains
    private ILITEPIDController mRollerPID;
    private ProfileGains kIntakeGains = new ProfileGains().p(0.001).i(0).d(0);

    //Constants
    private final double kWheelCircumference = 4 * Math.PI;
    private final double kMaxFalconSpeed = 6380;
    private final double kVelocityConversion = 2048 * 1000 * kWheelCircumference;

    public IntakeModule() {
        //TODO - add ids to below things
        //initialize motors and such
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kMAXIntakeRollerId);
        mArmSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,-1);

        //create pid values, set min/max input/outputs
        mRollerPID = new ILITEPIDController(ILITEPIDController.EPIDControlType.VELOCITY, kIntakeGains, clock);
        mRollerPID.setInputRange(-kMaxFalconSpeed, kMaxFalconSpeed);
        mRollerPID.setOutputRange(-kMaxFalconSpeed, kMaxFalconSpeed);
    }

    @Override
    public void readInputs() {
        db.cargo.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kVelocityConversion);
        db.cargo.set(PNEUMATIC_STATE, mArmSolenoid.get());
    }

    @Override
    public void setOutputs() {
        //calculate pid velocity
        double desiredVelocity = mRollerPID.calculate(db.cargo.get(ROLLER_VEL_ft_s), db.cargo.get(SET_ROLLER_VEL_ft_s));

        //set pid value to motor
        mIntakeRoller.set(TalonFXControlMode.Velocity, desiredVelocity);

        //turning the solenoids on or off
        setPneumaticIntake();
    }

    //if pneumatic state is 1/on, set solenoids on
    public void setPneumaticIntake() {
        if (db.cargo.get(PNEUMATIC_STATE) == 1.0) {
            mArmSolenoid.set(true);
        }

        if (db.cargo.get(PNEUMATIC_STATE) == 0.0) {
            mArmSolenoid.set(false);
        }
    }
}