package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EShooterSystemData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;

public class FlywheelModule extends Module {

    // Flywheel Motors
    private CANSparkMax mFeeder;
    private TalonFX mFlywheelFalconMaster;
    private TalonFX mFlywheelFalconFollower;
    private Servo mHoodServo;

    private Potentiometer mHoodPot;

    // Constants
    private static final double kHoodGearRatio = 32 / 20 * 14 / 360;

    private static final int FLYWHEEL_SLOT = 1;
    private static ProfileGains mFlywheelGains = new ProfileGains()
            .slot(FLYWHEEL_SLOT)
            .p(0.0005)
            ;


    private static final int SERVO_SLOT = 2;
    private static ProfileGains mServoGains = new ProfileGains()
            .slot(SERVO_SLOT)
            .p(0)
            .maxAccel(0)
            .maxVelocity(0)
            ;


    private double mIntegral = 0;
    private final int kFlywheelFalconPIDSlot = 0;

    public FlywheelModule() {
//        mFlywheelFalconMaster = new TalonFX(50);     //programming bot flywheel id

        mFlywheelFalconMaster = new TalonFX(Settings.Hardware.CAN.kFalconMasterId);
        mFlywheelFalconMaster.setNeutralMode(NeutralMode.Brake);
        mFlywheelFalconMaster.set(ControlMode);

        mFlywheelFalconFollower = new TalonFX(Settings.Hardware.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Brake);
        mFlywheelFalconFollower.setInverted(true);

        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);

        mHoodServo = new Servo(Settings.Hardware.CAN.kHoodServoId);

        mHoodPot = new AnalogPotentiometer(Settings.Hardware.Analog.kHoodPot);

//        mFlywheelFalconMaster.config_kP(kFlywheelFalconPIDSlot, 0.005);
//        mFlywheelFalconMaster.config_kI(kFlywheelFalconPIDSlot, 0);
//        mFlywheelFalconMaster.config_kD(kFlywheelFalconPIDSlot, 0);
//        mFlywheelFalconMaster.config_kF(kFlywheelFalconPIDSlot, 0);
    }
    
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
//        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }

    @Override
    public void readInputs(double pNow) {
        Distance distanceFromTarget = Distance.fromInches(Robot.DATA.limelight.get(ELimelightData.CALC_DIST_TO_TARGET));

//        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, DriveModule.Conversions.ticksPer100msToRotationsPerSecond(mFlywheelFalcon.getSelectedSensorVelocity()));

        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_FLYWHEEL_VELOCITY, mFlywheelFalconMaster.getSelectedSensorVelocity());

        if (Robot.DATA.limelight.isSet(ELimelightData.TV)) {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
        } else {
            Robot.DATA.flywheel.set(EShooterSystemData.FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
            Robot.DATA.flywheel.set(EShooterSystemData.SERVO_DISTANCE_BASED_ANGLE, 0);
        }

        Robot.DATA.flywheel.set(EShooterSystemData.CURRENT_POTENTIOMETER_TURNS, mHoodPot.get());
    }

    @Override
    public void setOutputs(double pNow) {
        hoodAnglePID(0.005, 0, 0);
        mFlywheelFalconMaster.set(ControlMode.Velocity, Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFalconFollower.set(ControlMode.Velocity);
        mFeeder.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_FEEDER_VELOCITY));

        mHoodServo.set(Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE));
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_SERVO_ANGLE, 0);
        Robot.DATA.flywheel.set(EShooterSystemData.TARGET_TURRET_VELOCITY, 0);
    }

    private boolean isPastVelocity(double pVelocity) {
        // TODO Convert ticks per 100 ms to rpm
        return mFlywheelFalconMaster.getSelectedSensorVelocity() >= pVelocity;
    }

    private double calcSpeedFromDistance(Distance pDistance) {
        //TODO figure out necessity
        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
                - 0.209 * Math.pow(pDistance.inches(), 2)
                + 6.31 * pDistance.inches()
                + 227;
    }

    private double calcAngleFromDistance(Distance pDistance) {
        //TODO tuning
        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
                + 0.157 * Math.pow(pDistance.inches(), 2)
                + 2.94 * pDistance.inches()
                + 68.2;
    }

    private double hoodAnglePID(double pP, double pI, double pD) {
        // TODO Convert into an Angle Measurement
        double error = pP * (Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE) - Robot.DATA.flywheel.get(EShooterSystemData.CURRENT_SERVO_ANGLE));
        double velocity = pD * mHoodServo.getSpeed();
        mIntegral += pI * mHoodServo.getSpeed();
        return error + mIntegral + velocity;
    }

    private void potentiometerPID() {
        double output = Robot.DATA.flywheel.get(EShooterSystemData.TARGET_SERVO_ANGLE) / 5;
    }
}