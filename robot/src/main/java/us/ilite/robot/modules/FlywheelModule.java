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
    public static final double kAcceleratorThreshold = 0.14;

    private final PIDGains kShooterGains = new PIDGains( 0.0005 ,0 , 0);
    private double mPreviousTime;

    private TalonFX mFlywheelMaster;
    private TalonSRX mFlywheelFeeder;
    private PigeonIMU mTurretGyro;

    private PIDController mShooterPIDMaster;
    private PIDController mShooterPIDFeeder;

    public FlywheelModule() {
        mFlywheelMaster = new TalonFX(50);
        mFlywheelFeeder = TalonSRXFactory.createPermanentSlaveTalon(51 , 50);
        mTurretGyro = new PigeonIMU(Settings.Hardware.CAN.kPigeonIDForFlywheel);
    }
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelMaster.getSelectedSensorVelocity());
        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FEEDER_VELOCITY , mFlywheelFeeder.getSelectedSensorVelocity());

    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelMaster.set(ControlMode.Velocity , Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY));
        mFlywheelFeeder.set(ControlMode.PercentOutput, Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_FEEDER_VELOCITY ) );
    }


    @Override
    public void shutdown(double pNow) {

    }

}