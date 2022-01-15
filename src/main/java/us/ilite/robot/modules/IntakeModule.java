package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.EPowerCellData.*;

public class IntakeModule extends Module{

    private CANSparkMax mIntakeOne;
    private CANSparkMax mIntakeConveyer;
    private PneumaticsControlModule mPneumaticsArm;

    private RelativeEncoder mIntakeEncoderOne;
    private RelativeEncoder mIntakeConveyerEncoder;

    private final int kMaxNeoVelocity = 5676;

    public IntakeModule() {
        mIntakeOne = SparkMaxFactory.createDefaultSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakeConveyer = SparkMaxFactory.createDefaultSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakeEncoderOne = mIntakeOne.getEncoder();
        mIntakeConveyerEncoder = mIntakeConveyer.getEncoder();
    }

    //sends the information to db that updates frequently
    @Override
    public void readInputs() {
        db.intake.set(INTAKE_VEL_ft_s, mIntakeEncoderOne.getVelocity()); //send data to db constantly
        db.intake.set(INTAKE_VEL_ft_s, mIntakeConveyerEncoder.getVelocity());
    }

    //sets the outputs for things in this module, what this module will do on robot
    @Override
    public void setOutputs() {
        mIntakeOne.set(db.intake.get(SET_INTAKE_VEL_ft_s) / kMaxNeoVelocity); //setting the motors to the variable given
        mIntakeConveyer.set(db.intake.get(SET_H_pct));
    }
}