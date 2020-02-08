package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private CANPIDController mArmController;

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

    private boolean allBeamsBroken;
    private int beamsNotBrokenCycles;



    //Constants

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

    private static final int UP_PID_SLOT_ID = 1;
    public static double P = 5.0e-4;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.000391419;

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
        INTAKE (0.75),
        STOP (0.0),
        REVERSE (-0.75);

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
        BROKEN (true),
        NOT_BROKEN(false);

        boolean broken;

        EIndexingState(boolean broken) {
            this.broken = broken;
        }

        public boolean getState(){
            return this.broken;
        }
    }

    public PowerCellModule() {
        mIntakeState = EIntakeState.STOP;
        mArmState = EArmState.DISENGAGED;
        mIndexingState = EIndexingState.NOT_BROKEN;

        mSerializer = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mConveyorMotorHorizontal = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonHorizontalID);
        mConveyorMotorVertical = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonVerticalID );
        mConveyorMotorVertical.setInverted(true);

        mArmMotor = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kArmNEOAdress , CANSparkMaxLowLevel.MotorType.kBrushless);

        mBeamBreaker1 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel1 );
        mBeamBreaker2 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel2 );
        mBeamBreaker3 = new DigitalBeamSensor( Settings.Hardware.DIO.kBeamChannel3 );
        mDigitalBeamSensors = new DigitalBeamSensor[]{mBeamBreaker1, mBeamBreaker2, mBeamBreaker3};

        mArmController = mSerializer.getPIDController();

        mIntakeEncoder = mSerializer.getEncoder();
        mArmEncoder = mArmMotor.getEncoder();

        mArmController.setP(P, UP_PID_SLOT_ID);
        mArmController.setI(I, UP_PID_SLOT_ID);
        mArmController.setD(D, UP_PID_SLOT_ID);

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        mArmController.setP(P);
        mArmController.setI(I);
        mArmController.setD(D);
        mArmController.setFF(F);
    }

    @Override
    public void readInputs(double pNow) {
//        mIntakeState = EIntakeState.values()[db.powercell.get(EPowerCellData.DESIRED_INTAKE_STATE).intValue()];

        db.powercell.set(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN, List.of(mDigitalBeamSensors).stream().filter(e -> !e.isBroken()).count());
        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY, mSerializer.getOutputCurrent());
        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S, mIntakeEncoder.getVelocity());
        db.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , mArmEncoder.getPosition());
        db.powercell.set(EPowerCellData.CURRENT_H_VELOCITY, mConveyorMotorHorizontal.getStatorCurrent());
        db.powercell.set(EPowerCellData.CURRENT_V_VELOCITY, mConveyorMotorVertical.getStatorCurrent());
        db.powercell.set(EPowerCellData.BREAK_SENSOR_1_STATE, (mBeamBreaker1.isBroken()));
        db.powercell.set(EPowerCellData.BREAK_SENSOR_2_STATE, (mBeamBreaker2.isBroken()));
        db.powercell.set(EPowerCellData.BREAK_SENSOR_3_STATE, (mBeamBreaker3.isBroken()));

        if(db.powercell.get(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN) >= 3.0){
            db.powercell.set(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN , (db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN )) + 1)  ;
        }
        else{
            db.powercell.set(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN , 3)  ;
        }

        double currentSensorsBroken = db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN);
        if (currentSensorsBroken < 3) {
            beamsNotBrokenCycles++;
        } else {
            beamsNotBrokenCycles = 0;
        }
        db.powercell.set(EPowerCellData.ALL_BEAMS_BROKEN, beamsNotBrokenCycles < 15);

        //TODO Determine Indexer State
    }

    @Override
    public void setOutputs(double pNow) {
        mSerializer.set(db.powercell.get(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S));
        mConveyorMotorHorizontal.set(ControlMode.PercentOutput, db.powercell.get(EPowerCellData.DESIRED_H_VELOCITY));
        mConveyorMotorVertical.set(ControlMode.PercentOutput, db.powercell.get(EPowerCellData.DESIRED_V_VELOCITY));
        mArmEncoder.setPosition(db.powercell.get(EPowerCellData.DESIRED_ARM_ANGLE));
    }

    @Override
    public void shutdown(double pNow) {
//        mIntakeState = EIntakeState.STOP;
//        mArmState = EArmState.DISENGAGED;
//        mIndexingState = EIndexingState.NOT_INDEXING;
    }

    public boolean isCurrentLimiting(){
        return db.pdp.get(EPowerDistPanel.CURRENT5) > kWarnCurrentLimitThreshold;
    }

    public void startIndexing() {
        //TODO determine V_Motor and H_Motor specifics with Beam breaker
        mBeamCountBroken = (int) List.of(mDigitalBeamSensors).stream().filter(e -> !e.isBroken()).count();

        SmartDashboard.putNumber("BeamCountBroken" , mBeamCountBroken);
        SmartDashboard.putNumber("BeamCountBrokenGoal" , mGoalBeamCountBroken);


//        for (DigitalBeamSensor mDigitalBeamSensor : mDigitalBeamSensors) {
//            if (mDigitalBeamSensor.isBroken()) mBeamCountBroken++;
//        }
        if ( mBeamCountBroken < mGoalBeamCountBroken) {
            mConveyorMotorHorizontal.set( ControlMode.PercentOutput, db.powercell.get(EPowerCellData.DESIRED_H_VELOCITY) );
        } else {
            mConveyorMotorHorizontal.set( ControlMode.PercentOutput, 0.0 );
        }

        mGoalBeamCountBroken = mBeamCountBroken + 1;
    }
    public void setIntakeState(EIntakeState pIntakeState){
        mIntakeState = pIntakeState;
    }
}