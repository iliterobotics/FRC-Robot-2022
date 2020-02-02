package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.util.List;

public class PowerCellModule extends Module {

    //Change in drivetrain velocity to control intake speed
    public static Double kDeltaIntakeVel = 0d;

    private final CANPIDController mCanController;

    // Intake Motors
    private CANSparkMax mIntakeMotor;
    private TalonSRX mConveyorMotorH;
    private TalonSRX mConveyorMotorV;

    //Arm
    private CANSparkMax mArmMotor;

//    Beam Breakers
//    private DigitalBeamSensor mBeamBreaker0;
//    private DigitalBeamSensor mBeamBreaker1;
//    private DigitalBeamSensor mBeamBreaker2;

    //Array of beam-breakers (Used when indexing PowerCells)
    private DigitalBeamSensor[] mDigitalBeamSensors;

    public static double kIntakeTalonPower = 1d;
    public static double kForStopTalon = 0d;
    public static double kIntakePower = 1.0;
    public static double kForStop = 1.0;
    public static double kPowerCellP;
    public static double kPowerCellI;
    public static double kPowerCellD;
    public static double kPowerCellF;
    public static double kWarnCurrentLimitThreshold = 30; //Tune this later
    public static double kArmPowerEngaged = 1.0;
    public static double kArmPowerDisengaged = 0.0;
    //Intake state
    private EIntakeState mIntakeState;
    private EIndexingState mIndexingState;

    //Arm State
    private EArmState mArmState;
    
    //For indexing
    private int mGoalBeamCountBroken = 0;
    private int mBeamCountBroken = 0;


    private ILog mLog = Logger.createLog(this.getClass());

    private CANEncoder mIntakeEncoder;
    private CANEncoder mArmEncoder;

    public enum EIntakeState {
        //TODO find velocities
        INTAKE (1.0),
        STOP (0.0),
        REVERSE (-1.0);

        double power;

        EIntakeState (double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public enum EArmState {
        //TODO find angles
        ENGAGED (1.0),
        DISENGAGED (0.0);

        double angle;

        EArmState (double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public enum EIndexingState {
        MOVING (1.0),
        NOT_MOVING(0.0);

        double power;

        EIndexingState(double power) {
            this.power = power;
        }
    }

    public PowerCellModule() {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mIndexingState = EIndexingState.NOT_MOVING;

        mIntakeMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mConveyorMotorH = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonOneID );
        mConveyorMotorV = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonTwoID );

        mArmMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kArmNEOAdress , CANSparkMaxLowLevel.MotorType.kBrushless);

//        mBeamBreaker0 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel0 );
//        mBeamBreaker1 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel1 );
//        mBeamBreaker2 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel2 );
//        mDigitalBeamSensors = new DigitalBeamSensor[]{mBeamBreaker0, mBeamBreaker1, mBeamBreaker2};

        mCanController = mIntakeMotor.getPIDController();

        mIntakeEncoder = mIntakeMotor.getEncoder();
        mArmEncoder = mArmMotor.getEncoder();

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        mCanController.setP(kPowerCellP);
        mCanController.setI(kPowerCellI);
        mCanController.setD(kPowerCellD);
        mCanController.setFF(kPowerCellF);
    }

    @Override
    public void readInputs(double pNow) {
//        mIntakeState = EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.DESIRED_INTAKE_STATE).intValue()];

        Robot.DATA.powercell.set(EPowerCellData.CURRENT_INTAKE_POWER_PCT, mIntakeMotor.getOutputCurrent());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S, mIntakeEncoder.getVelocity());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , mArmEncoder.getPosition());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_H_POWER_PCT, mConveyorMotorH.getStatorCurrent());
        Robot.DATA.powercell.set(EPowerCellData.CURRENT_V_POWER_PCT, mConveyorMotorV.getStatorCurrent());
//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_0 , readBeamBreakerState(mBeamBreaker0.isBroken()));
//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_1 , readBeamBreakerState(mBeamBreaker1.isBroken()));
//        Robot.DATA.powercell.set(EPowerCellData.BREAK_SENSOR_2 , readBeamBreakerState(mBeamBreaker2.isBroken()));

        //TODO Determine Indexer State
    }

    @Override
    public void setOutputs(double pNow) {
//        mCANMotor.set(EIntakeState.values()[Robot.DATA.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());

        mBeamCountBroken = (int) List.of(mDigitalBeamSensors).stream().map(DigitalBeamSensor::isBroken).filter(e -> e).count();
        //TODO determine V_Motor and H_Motor specifics with Beam breaker
        if ( mBeamCountBroken < mGoalBeamCountBroken) {
            mConveyorMotorH.set( ControlMode.Velocity, Robot.DATA.powercell.get(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S) );
        } else {
            mConveyorMotorH.set( ControlMode.Velocity, 0.0 );
        }

//        mIntakeMotor.set(Robot.DATA.powercell.get(EPowerCellData.DESIRED_INTAKE_POWER_PCT));
        mIntakeMotor.set(Robot.DATA.powercell.get(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S));
        mConveyorMotorH.set(ControlMode.PercentOutput, Robot.DATA.powercell.get(EPowerCellData.DESIRED_H_POWER_PCT));
        mConveyorMotorV.set(ControlMode.PercentOutput, Robot.DATA.powercell.get(EPowerCellData.DESIRED_H_POWER_PCT));
        mArmEncoder.setPosition(Robot.DATA.powercell.get(EPowerCellData.DESIRED_ARM_ANGLE));

//        mGoalBeamCountBroken = mBeamCountBroken + 1;
    }

    @Override
    public void shutdown(double pNow) {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mIndexingState = EIndexingState.NOT_MOVING;
    }

    private double readBeamBreakerState(boolean isBroken){
        if ( isBroken ) {
            return 1.0;
        }
        return 0.0;
    }

    public boolean isCurrentLimiting(){
        return Robot.DATA.pdp.get(EPowerDistPanel.CURRENT5) > kWarnCurrentLimitThreshold;
    }


}
