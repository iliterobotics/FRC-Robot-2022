package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class PowerCellModule extends Module {

    //Change in drivetrain velocity to control intake speed
    public static double kDeltaIntakeVel = 0d;
    private Data db = Robot.DATA;


    // Serializer Motors
    private TalonSRX mConveyorMotorHorizontal;
    private TalonSRX mConveyorMotorVertical;

    //Arm
    private CANSparkMax mIntakePivot;
    private CANSparkMax mIntakeRoller;
    private CANPIDController mIntakePivotCtrl;
    private CANEncoder mIntakePivotEncoder;
    private CANEncoder mIntakeRollerEncoder;

//    Beam Breakers
    private DigitalBeamSensor mEntryBeam;
    private DigitalBeamSensor mSecondaryBeam;
    private DigitalBeamSensor mExitBeam;

    private boolean allBeamsBroken;
    private int mEntryBeamNotBrokenCycles = 0;
    private int mSecondaryBeamNotBrokenCycles = 0;
    private int mExitBeamNotBrokenCycles = 0;



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

    private static final int INTAKE_PIVOT_SLOT = 1;
    private static final ProfileGains mIntakePivotGains = new ProfileGains()
            .slot(INTAKE_PIVOT_SLOT)
            .p(0.0005)
            .f(0.0000391419);

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


    public enum EIntakeState {
        //TODO find velocities
        INTAKE (0.1),
        STOP (0.0),
        REVERSE (-0.1);

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

        mIntakeRoller = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mConveyorMotorHorizontal = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonPowerCellSerializerBAG);
        mConveyorMotorVertical = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonVerticalID );
        mConveyorMotorVertical.setInverted(true);

        mIntakePivot = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kNEOIntakeArm, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEntryBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kEntryBeamChannel);
        mSecondaryBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kSecondaryBeamChannel);
        mExitBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kExitBeamChannel);
        mDigitalBeamSensors = new DigitalBeamSensor[]{mEntryBeam, mSecondaryBeam, mExitBeam};

        mIntakePivotEncoder = mIntakePivot.getEncoder();
        mIntakeRollerEncoder = mIntakeRoller.getEncoder();

        mIntakePivotCtrl = mIntakePivot.getPIDController();
        HardwareUtils.setGains(mIntakePivotCtrl, mIntakePivotGains);
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        HardwareUtils.setGains(mIntakePivotCtrl, mIntakePivotGains);
    }

    @Override
    public void readInputs(double pNow) {
//        mIntakeState = EIntakeState.values()[db.powercell.get(EPowerCellData.DESIRED_INTAKE_STATE).intValue()];
        Object[] brokenArray = Arrays.stream(mDigitalBeamSensors).map(e -> !e.isBroken()).toArray();

        db.powercell.set(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN, List.of(mDigitalBeamSensors).stream().filter(e -> !e.isBroken()).count());
        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY, mIntakeRoller.getOutputCurrent());
        db.powercell.set(EPowerCellData.CURRENT_INTAKE_VELOCITY_FT_S, mIntakeRollerEncoder.getVelocity());
        db.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , mIntakePivotEncoder.getPosition());
        db.powercell.set(EPowerCellData.CURRENT_H_VELOCITY, mConveyorMotorHorizontal.getStatorCurrent());
        db.powercell.set(EPowerCellData.CURRENT_V_VELOCITY, mConveyorMotorVertical.getStatorCurrent());

        if(db.powercell.get(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN) >= 3.0){
            db.powercell.set(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN , (db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN )) + 1)  ;
        }
        else{
            db.powercell.set(EPowerCellData.DESIRED_AMOUNT_OF_SENSORS_BROKEN , 3)  ;
        }

//        double currentSensorsBroken = db.powercell.get(EPowerCellData.CURRENT_AMOUNT_OF_SENSORS_BROKEN);
//        if (currentSensorsBroken < 3) {
//            mEntryBeamNotBrokenCycles++;
//        } else {
//            mEntryBeamNotBrokenCycles = 0;
//    }
//        db.powercell.set(EPowerCellData.ALL_BEAMS_BROKEN, mEntryBeamNotBrokenCycles < 15);

        if((boolean)brokenArray[0]) {
            mEntryBeamNotBrokenCycles++;
        } else {
            mEntryBeamNotBrokenCycles = 0;
        }

        if((boolean)brokenArray[1]) {
            mSecondaryBeamNotBrokenCycles++;
        } else {
            mSecondaryBeamNotBrokenCycles = 0;
        }

        if((boolean)brokenArray[2]) {
            mExitBeamNotBrokenCycles++;
        } else {
            mExitBeamNotBrokenCycles = 0;
        }

        db.powercell.set(EPowerCellData.ENTRY_BEAM_BREAKER, mEntryBeamNotBrokenCycles > 15);//mEntryBeamNotBrokenCycles > 15);
        db.powercell.set(EPowerCellData.SECONDARY_BREAM_BREAKER, mSecondaryBeamNotBrokenCycles > 15);
        db.powercell.set(EPowerCellData.EXIT_BEAM_BREAKER, mExitBeamNotBrokenCycles > 15);

        //TODO Determine Indexer State
    }

    @Override
    public void setOutputs(double pNow) {
        mIntakeRoller.set(db.powercell.get(EPowerCellData.DESIRED_INTAKE_VELOCITY_FT_S));
        mConveyorMotorHorizontal.set(ControlMode.PercentOutput, db.powercell.get(EPowerCellData.DESIRED_H_VELOCITY));
        mConveyorMotorVertical.set(ControlMode.PercentOutput, db.powercell.get(EPowerCellData.DESIRED_V_VELOCITY));
        mIntakePivotEncoder.setPosition(db.powercell.get(EPowerCellData.DESIRED_ARM_ANGLE));
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