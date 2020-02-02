package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.PIDGains;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;


public class FlywheelModule extends Module {
   // public static final double kAcceleratorThreshold = 0.14;

    private TalonSRX mFlywheelFeederOne;
    private TalonFX mFlywheelMaster;
    private TalonSRX mFlywheelFeederTwo;
    private Servo mServo;
    //Add SparkMax later
    private PigeonIMU mTurretGyro;

    public FlywheelModule() {
        mFlywheelMaster = new TalonFX(50);
        mFlywheelFeederTwo = TalonSRXFactory.createPermanentSlaveTalon(51 , 50);
        mFlywheelFeederOne= TalonSRXFactory.createPermanentSlaveTalon(Settings.Hardware.CAN.kShooterID , 50);
        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kPigeonIDForFlywheel);
    }
    public enum EFlywheelState{
        SHOOTING(1700),
        REVERSE(-1700),
        STOP(0);

        double velocity;
        EFlywheelState(double velocity){
            this.velocity = velocity;
        }
    }
    public enum EHoodState{
        //TODO find the states of this
    }
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelMaster.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FEEDER_VELOCITY , mFlywheelFeederOne.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FEEDER_VELOCITY , mFlywheelFeederTwo.getSelectedSensorVelocity());

    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMaster.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFeederOne.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY ) );
        mFlywheelFeederTwo.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));
    }


    @Override
    public void shutdown(double pNow) {

    }

}