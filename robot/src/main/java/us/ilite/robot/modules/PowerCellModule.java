package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EIntake;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import javax.naming.ldap.Control;

public class PowerCellModule extends Module {

    // Intake Motors
    private CANSparkMax mCANMotor;
    private TalonSRX mTalonOne;
    private TalonSRX mTalonTwo;
    private TalonSRX mTalonThree;

    //Beam Breakers
    private DigitalBeamSensor mBeamBreaker1;
    private DigitalBeamSensor mBeamBreaker2;
    private DigitalBeamSensor mBeamBreaker3;

    private Data mData;

    //Intake state
    private EIntakeState mIntakeState;

    private ILog mLog = Logger.createLog(this.getClass());

    public enum EIntakeState {
        INTAKE(1.0),
        STOP (0.0),
        REVERSE (-1.0);

        private double power;

        EIntakeState (double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public PowerCellModule( Data pData ) {
        mData = pData;

        mCANMotor = SparkMaxFactory.createDefaultSparkMax( Settings.kCANIntakeID , CANSparkMaxLowLevel.MotorType.kBrushless );
        mTalonOne = TalonSRXFactory.createDefaultTalon( Settings.kTalonOneID );
        mTalonTwo = TalonSRXFactory.createDefaultTalon( Settings.kTalonTwoID );
        mTalonThree = TalonSRXFactory.createDefaultTalon( Settings.kTalonThreeID );

        mBeamBreaker1 = new DigitalBeamSensor( Settings.kBeamChannel1);
        mBeamBreaker2 = new DigitalBeamSensor( Settings.kBeamChannel2);
        mBeamBreaker3 = new DigitalBeamSensor( Settings.kBeamChannel3);

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        mData.powercell.set(EPowerCellData.DESIRED_INTAKE_POWER_PCT , mCANMotor.getOutputCurrent());
        mData.powercell.set(EPowerCellData.DESIRED_CONVEYOR_POWER_PCT , mTalonOne.getOutputCurrent());
        mData.powercell.set(EPowerCellData.DESIRED_SERLIALIZER_POWER_PCT ,mTalonTwo.getOutputCurrent() );

        mData.powercell.set(EPowerCellData.BREAK_SENSOR_0 , readBeamBreakerState(mBeamBreaker1.isBroken()));
        mData.powercell.set(EPowerCellData.BREAK_SENSOR_1 , readBeamBreakerState(mBeamBreaker2.isBroken()));
        mData.powercell.set(EPowerCellData.BREAK_SENSOR_2 , readBeamBreakerState(mBeamBreaker3.isBroken()));

    }

    @Override
    public void setOutputs(double pNow) {
        mCANMotor.set(EIntakeState.values()[mData.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonOne.set(ControlMode.PercentOutput, EIntakeState.values()[mData.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonTwo.set(ControlMode.PercentOutput, EIntakeState.values()[mData.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        mTalonThree.set(ControlMode.PercentOutput, EIntakeState.values()[mData.powercell.get(EPowerCellData.CURRENT_POWERCELL_STATE).intValue()].getPower());
        //May want to add beam breakers
    }

    @Override
    public void shutdown(double pNow) {
        mCANMotor.set(0.0);
        mTalonOne.set(ControlMode.PercentOutput, 0d);
        mTalonTwo.set(ControlMode.PercentOutput, 0d);
        mTalonThree.set(ControlMode.PercentOutput, 0d);
    }
    public void setDesiredIntakeState(EIntakeState pDesiredState){
        mIntakeState = pDesiredState;
    }

    public EIntakeState returnIntakeState(){
        return this.mIntakeState;
    }
    
    public void startIntaking(){
        setDesiredIntakeState(EIntakeState.INTAKE);
    }

    public void unjam(){
        setDesiredIntakeState(EIntakeState.REVERSE);
    }

    public double readBeamBreakerState(boolean isBroken){
        if ( isBroken ){
            return 1.0;
        }
        return 0.0;
    }


}
