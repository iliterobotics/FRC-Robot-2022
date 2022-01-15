package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.hardware.SolenoidWrapper;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EPowerCellData.*;

public class IntakeModule extends Module{

    private CANSparkMax mIntakeRoller;
    private CANSparkMax mIntakeConveyor;

    private Solenoid mArmSolenoidLeft;
    private Solenoid mArmSolenoidRight;

    private RelativeEncoder mIntakeEncoderOne;
    private RelativeEncoder mIntakeConveyorEncoder;

    private final int kMaxNeoVelocity = 5676;

    public IntakeModule() {
        mIntakeRoller = SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakeConveyor = SparkMaxFactory.createDefaultSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakeEncoderOne = mIntakeRoller.getEncoder();
        mIntakeConveyorEncoder = mIntakeConveyor.getEncoder();

        mArmSolenoidLeft = new Solenoid(PneumaticsModuleType.CTREPCM,-1);
        mArmSolenoidRight = new Solenoid(PneumaticsModuleType.CTREPCM, -1);
    }

    //sends the information to db that updates frequently
    @Override
    public void readInputs() {
        db.intake.set(INTAKE_VEL_ft_s, mIntakeEncoderOne.getVelocity()); //send data to db constantly
        db.intake.set(INTAKE_VEL_ft_s, mIntakeConveyorEncoder.getVelocity());

        db.intake.set(LEFT_PNEUMATIC_STATE, mArmSolenoidLeft.get());
        db.intake.set(RIGHT_PNEUMATIC_STATE, mArmSolenoidRight.get());
    }

    //sets the outputs for things in this module, what this module will do on robot
    @Override
    public void setOutputs() {
        mIntakeConveyor.set(db.intake.get(SET_INTAKE_VEL_ft_s)); //setting the motors to the variable given
        mIntakeRoller.set(db.intake.get(SET_H_pct));

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