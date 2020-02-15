package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import static us.ilite.common.types.EPowerCellData.*;
import us.ilite.common.types.sensor.EPowerDistPanel;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import java.util.Arrays;
import java.util.List;

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
    public static int kWarnCurrentLimitThreshold = 20;

    private static final int INTAKE_PIVOT_DOWN_SLOT = 1;
    private static final int INTAKE_PIVOT_UP_SLOT = 2;

    // 100:1 AM Sport with 16:36 sprocket reduction
    // We may change this ratio to speed up the arm, so leave the math separated out like this
    private static final double kPivotGearRatio = 1.0/100.0 * 16.0 / 36.0;
    // 36T Driving sprocket on the pivot, 16T driven sprocket on the cross-axle
    private static final double kAbsoluteEncoderRatio = kPivotGearRatio * 36.0 / 16.0;
    // Rotations in degrees
    private static final double kPivotConversion = kPivotGearRatio * 360.0;
    // RPM to degrees / second
    private static final double kPivotVelocityConversion = kPivotConversion / 60.0;
    private static final double kMaxIntakePivotVelocityDeg_s = 10.0;
    private static final ProfileGains mIntakePivotDownGains = new ProfileGains()
            .slot(INTAKE_PIVOT_DOWN_SLOT)
            .p(0.00025)
            .maxAccel(9000d)
            .maxVelocity(6000d)
            ;

    private static final ProfileGains mIntakePivotUpGains = mIntakePivotDownGains;
//    private static final ProfileGains mIntakePivotUpGains = new ProfileGains()
//            .slot(INTAKE_PIVOT_UP_SLOT)
//            .p(0.0005)
//            .f(0.1)
////            .maxAccel(kMaxIntakePivotVelocityDeg_s * kPivotVelocityConversion/4.0)
////            .maxVelocity(kMaxIntakePivotVelocityDeg_s / kPivotVelocityConversion / 2.0)
//            .maxAccel(6000d)
//            .maxVelocity(6000d)
////            .velocityConversion(kPivotVelocityConversion)
////            .positionConversion(kPivotConversion)
////            .f(0.0000391419)
//            ;

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
        NONE(0.0, 0),
        OUT(90.0, INTAKE_PIVOT_DOWN_SLOT),
        // TODO - fix to UP slot
        STOW(0.0, INTAKE_PIVOT_DOWN_SLOT),
        HOLD(0.0, INTAKE_PIVOT_DOWN_SLOT);

        public double angle;
        public int slot;

        EArmState (double angle, int slot) {
            this.angle = angle;
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
        mArmState = EArmState.STOW;
        mIndexingState = EIndexingState.NOT_BROKEN;

        mIntakeRoller = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kMAXIntakeRollerId, CANSparkMaxLowLevel.MotorType.kBrushless );
        mIntakeRoller.setInverted(true);
        mIntakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mIntakeRoller.setSmartCurrentLimit(25);

        mConveyorMotorHorizontal = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonPowerCellSerializer);
        mConveyorMotorVertical = TalonSRXFactory.createDefaultTalon( Settings.Hardware.CAN.kTalonVerticalID );
        mConveyorMotorVertical.setInverted(true);

        mIntakePivot = SparkMaxFactory.createDefaultSparkMax( Settings.Hardware.CAN.kMAXIntakeArm, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakePivot.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mIntakePivot.setSecondaryCurrentLimit(15);

        mEntryBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kEntryBeamChannel);
        mSecondaryBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kSecondaryBeamChannel);
        mExitBeam = new DigitalBeamSensor( Settings.Hardware.DIO.kExitBeamChannel);
        mDigitalBeamSensors = new DigitalBeamSensor[]{mEntryBeam, mSecondaryBeam, mExitBeam};

        mIntakePivotEncoder = new CANEncoder(mIntakePivot);
        mIntakeRollerEncoder = mIntakeRoller.getEncoder();

        mIntakePivotCtrl = mIntakePivot.getPIDController();
        HardwareUtils.setGains(mIntakePivotCtrl, mIntakePivotDownGains);
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        HardwareUtils.setGains(mIntakePivotCtrl, mIntakePivotUpGains);
//        HardwareUtils.setGains(mIntakePivotEncoder, mIntakePivotUpGains);
        HardwareUtils.setGains(mIntakePivotCtrl, mIntakePivotDownGains);
//        HardwareUtils.setGains(mIntakePivotEncoder, mIntakePivotDownGains);
        mIntakePivotEncoder.setPosition(0.0);
        mIntakePivotCtrl.setOutputRange(0.0, 95.0);
        SmartDashboard.putNumber("Rotation Conversion (deg)", mIntakePivotDownGains.POSITION_CONVERSION_FACTOR);
        SmartDashboard.putNumber("Max Rotation Speed (deg/s)", kMaxIntakePivotVelocityDeg_s * kPivotVelocityConversion);
    }

    @Override
    public void readInputs(double pNow) {
//        mIntakeState = EIntakeState.values()[db.powercell.get(EPowerCellData.DESIRED_INTAKE_STATE).intValue()];
        Object[] brokenArray = Arrays.stream(mDigitalBeamSensors).map(e -> !e.isBroken()).toArray();

        db.powercell.set(CURRENT_AMOUNT_OF_SENSORS_BROKEN, List.of(mDigitalBeamSensors).stream().filter(e -> !e.isBroken()).count());
        db.powercell.set(INTAKE_ROLLER_CURRENT, mIntakeRoller.getOutputCurrent());
        db.powercell.set(CURRENT_INTAKE_VELOCITY_FT_S, mIntakeRollerEncoder.getVelocity());
        db.powercell.set(CURRENT_ARM_ANGLE , mIntakePivotEncoder.getPosition() * kPivotConversion);
        db.powercell.set(SERIALIZER_CURRENT, mConveyorMotorHorizontal.getStatorCurrent());
        db.powercell.set(VERTICAL_CURRENT, mConveyorMotorVertical.getStatorCurrent());
        db.powercell.set(INTAKE_PIVOT_CURRENT, mIntakePivot.getOutputCurrent());

        if(db.powercell.get(DESIRED_AMOUNT_OF_SENSORS_BROKEN) >= 3.0){
            db.powercell.set(DESIRED_AMOUNT_OF_SENSORS_BROKEN , (db.powercell.get(CURRENT_AMOUNT_OF_SENSORS_BROKEN )) + 1)  ;
        }
        else{
            db.powercell.set(DESIRED_AMOUNT_OF_SENSORS_BROKEN , 3)  ;
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

        db.powercell.set(ENTRY_BEAM, mEntryBeamNotBrokenCycles > 15);//mEntryBeamNotBrokenCycles > 15);
        db.powercell.set(SECONDARY_BREAM, mSecondaryBeamNotBrokenCycles > 15);
        db.powercell.set(EXIT_BEAM, mExitBeamNotBrokenCycles > 15);

        //TODO Determine Indexer State
    }

    @Override
    public void setOutputs(double pNow) {
        mConveyorMotorHorizontal.set(ControlMode.PercentOutput, db.powercell.get(DESIRED_H_VELOCITY));
        mConveyorMotorVertical.set(ControlMode.PercentOutput, db.powercell.get(DESIRED_V_VELOCITY));
        if(db.powercell.isSet(INTAKE_STATE)) {
            mIntakeRoller.set(db.powercell.get(DESIRED_INTAKE_VELOCITY_FT_S));
            EArmState state = db.powercell.get(INTAKE_STATE, EArmState.class);
            switch(state) {
                case OUT:
                case STOW:
                    mIntakePivotCtrl.setReference(state.angle/kPivotConversion, ControlType.kSmartMotion, INTAKE_PIVOT_DOWN_SLOT, 0.01);
                    break;
                default:
                    mIntakePivot.set(0.0);
            }
            SmartDashboard.putNumber("TARGET ANGLE (deg)", state.angle);
        } else {
            mIntakePivot.set(0.0);
        }
    }

    @Override
    public void shutdown(double pNow) {
//        mIntakeState = EIntakeState.STOP;
//        mArmState = EArmState.DISENGAGED;
//        mIndexingState = EIndexingState.NOT_INDEXING;
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
}