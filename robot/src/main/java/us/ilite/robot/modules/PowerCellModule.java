package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
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
    public static double kDeltaIntakeVel = 0d;
    private Data db = Robot.DATA;

    private CANPIDController mCanController;

    // Intake Motors
    private CANSparkMax mSerializer;
    private TalonSRX mConveyorMotorHorizontal;
    private TalonSRX mConveyorMotorVertical;

    //Arm
    private CANSparkMax mArmMotor;

//    Beam Breakers
    private DigitalBeamSensor mBeamBreaker1;
    private DigitalBeamSensor mBeamBreaker2;
    private DigitalBeamSensor mBeamBreaker3;



    ////////////////////Constants\\\\\\\\\\\\\\\\\\\

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
    //Array of beam-breakers (Used when indexing PowerCells)
    private DigitalBeamSensor[] mDigitalBeamSensors;


    private ILog mLog = Logger.createLog(this.getClass());

    private CANEncoder mIntakeEncoder;
    private CANEncoder mArmEncoder;

    public enum EIntakeState {
        //TODO find velocities
        INTAKE (1.0),
        STOP (0.0),
        REVERSE (-1.0);

        double pPower;

        EIntakeState (double pPower) {
            this.pPower = pPower;
        }

        public double getPower() {
            return pPower;
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
        INDEXING (1.0),
        NOT_INDEXING(0.0);

        double power;

        EIndexingState(double power) {
            this.power = power;
        }
    }

    public PowerCellModule() {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mIndexingState = EIndexingState.NOT_INDEXING;

        mSerializer = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mConveyorMotorHorizontal = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonOneID );
        mConveyorMotorVertical = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonTwoID );

        mArmMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kArmNEOAdress , CANSparkMaxLowLevel.MotorType.kBrushless);

//        mBeamBreaker1 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel1 );
//        mBeamBreaker2 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel2 );
//        mBeamBreaker3 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel3 );
//        mDigitalBeamSensors = new DigitalBeamSensor[]{mBeamBreaker1, mBeamBreaker2, mBeamBreaker3};

        mCanController = mSerializer.getPIDController();

        mIntakeEncoder = mSerializer.getEncoder();
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
//        mIntakeState = EIntakeState.values()[db.powercell.get(EPowerCellData.DESIRED_INTAKE_STATE).intValue()];

        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY, mSerializer.getOutputCurrent());
        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S, mIntakeEncoder.getVelocity());
        db.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , mArmEncoder.getPosition());
        db.powercell.set(EPowerCellData.CURRENT_H_VELOCITY, mConveyorMotorHorizontal.getStatorCurrent());
        db.powercell.set(EPowerCellData.CURRENT_V_VELOCITY, mConveyorMotorVertical.getStatorCurrent());
//        db.powercell.set(EPowerCellData.BREAK_SENSOR_0_STATE , readBeamBreakerState(mBeamBreaker1.isBroken()));
//        db.powercell.set(EPowerCellData.BREAK_SENSOR_1_STATE , readBeamBreakerState(mBeamBreaker1.isBroken()));
//        db.powercell.set(EPowerCellData.BREAK_SENSOR_2_STATE , readBeamBreakerState(mBeamBreaker2.isBroken()));

        //TODO Determine Indexer State
    }

    @Override
    public void setOutputs(double pNow) {
        mSerializer.set(db.powercell.get(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S));
        mConveyorMotorHorizontal.set(ControlMode.Velocity, db.powercell.get(EPowerCellData.CURRENT_H_VELOCITY));
        mConveyorMotorVertical.set(ControlMode.Velocity, db.powercell.get(EPowerCellData.CURRENT_V_VELOCITY));
        mArmEncoder.setPosition(db.powercell.get(EPowerCellData.CURRENT_ARM_ANGLE));
//        isCurrentLimiting();
//        startIndexing();
    }

    @Override
    public void shutdown(double pNow) {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mIndexingState = EIndexingState.NOT_INDEXING;
    }

    private double readBeamBreakerState(boolean isBroken){
        if ( isBroken ) {
            return 1.0;
        }
        return 0.0;
    }

    public boolean isCurrentLimiting(){
        return db.pdp.get(EPowerDistPanel.CURRENT5) > kWarnCurrentLimitThreshold;
    }
    public void startIndexing() {
        //TODO determine V_Motor and H_Motor specifics with Beam breaker
        mBeamCountBroken = (int) List.of(mDigitalBeamSensors).stream().map(DigitalBeamSensor::isBroken).filter(e -> e).count();
        if ( mBeamCountBroken < mGoalBeamCountBroken) {
            mConveyorMotorHorizontal.set( ControlMode.Velocity, db.powercell.get(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S) );
        } else {
            mConveyorMotorHorizontal.set( ControlMode.Velocity, 0.0 );
        }
        mGoalBeamCountBroken = mBeamCountBroken + 1;
    }
}