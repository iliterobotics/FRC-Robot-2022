package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;

import static us.ilite.common.types.EIntakeData.*;

public class IntakeModule extends Module{

    private TalonFX mIntakeRoller;
    private TalonFX mIntakeConveyor;

    private Solenoid mArmSolenoidLeft;
    private Solenoid mArmSolenoidRight;

    private PIDController mIntakePID;

    private final int kMaxNeoVelocity = 5676;//change to falcon
    private final double kWheelCircumference = 1.5;
    private final double kVelocityConversion = 2048 * 1000 * kWheelCircumference;

    private ProfileGains kIntakeGains = new ProfileGains().p(0.001).i(0).d(0);

    public IntakeModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kINRoller);
        mIntakeConveyor = new TalonFX(Settings.HW.CAN.kINFeeder);

        mArmSolenoidLeft = new Solenoid(PneumaticsModuleType.CTREPCM,-1);
        mArmSolenoidRight = new Solenoid(PneumaticsModuleType.CTREPCM, -1);

        mIntakePID = new PIDController(kIntakeGains, -kMaxNeoVelocity, kMaxNeoVelocity, clock.dt()); //clock.dt may not be correct time value
    }

    //sends the information to db that updates frequently
    @Override
    public void readInputs() {
        //need conversion
        db.intake.set(ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kVelocityConversion); //send data to db constantly
        db.intake.set(ROLLER_VEL_ft_s, mIntakeConveyor.getSelectedSensorVelocity()* kVelocityConversion);

        db.intake.set(LEFT_PNEUMATIC_STATE, mArmSolenoidLeft.get());
        db.intake.set(RIGHT_PNEUMATIC_STATE, mArmSolenoidRight.get());
    }

    //sets the outputs for things in this module, what this module will do on robot
    @Override
    public void setOutputs() {
        double desiredVelocity = mIntakePID.getOutput();
        //setting the motors to the variable given
        mIntakeRoller.set(TalonFXControlMode.Velocity, db.intake.get(SET_ROLLER_VEL_ft_s));
        mIntakeConveyor.set(TalonFXControlMode.PercentOutput, db.intake.get(SET_CONVEYOR_pct));


        setPneumaticIntake();
    }

    public void setPneumaticIntake() {
        //convert the double values of the pneumatic states to booleans
        if (db.intake.get(LEFT_PNEUMATIC_STATE) == 1.0) {
            mArmSolenoidLeft.set(true);
        }
        mArmSolenoidLeft.set(false);

        if (db.intake.get(RIGHT_PNEUMATIC_STATE) == 1.0) {
            mArmSolenoidRight.set(true);
        }
        mArmSolenoidRight.set(false);
    }
}