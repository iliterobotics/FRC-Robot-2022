package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

import static us.ilite.common.types.EShooterSystemData.*;

public class FlywheelModule extends Module {

    // Flywheel Motors
    private TalonFX mFlywheelFalconMaster;
    private TalonFX mFlywheelFalconFollower;
    private CANSparkMax mFeeder;
    private TalonSRX mTurret;
    private Servo mHoodServo;

    private Potentiometer mHoodPot;

    // Constants
    private static final double kHoodGearRatio = 32.0 / 20.0 * 14.0 / 360.0;//First stage was 32/20 in code but 36/20 in integration
    private static final double kTurretGearRatio = 9.0 / 72.0 * 18.0 / 160.0;
    private static final double kFlyWheelGearRatio = 36.0 / 24.0;

    private static final double kHoodConversionFactor = 0;  //TODO to be determined, 0-5 pot turns to high and low angle

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

        mFlywheelFalconFollower = new TalonFX(Settings.Hardware.CAN.kFalconFollowerId);
        mFlywheelFalconFollower.setNeutralMode(NeutralMode.Brake);
        mFlywheelFalconFollower.setInverted(true);

        mFeeder = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);

        mTurret = TalonSRXFactory.createDefaultTalon(Settings.Hardware.CAN.kSRXTurretId);

        mHoodServo = new Servo(Settings.Hardware.CAN.kHoodServoId);

        mHoodPot = new AnalogPotentiometer(Settings.Hardware.Analog.kHoodPot);

    }
    
    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        //Set PID gains & zero encoders here
    }

    @Override
    public void readInputs(double pNow) {
        Robot.DATA.flywheel.set(CURRENT_FLYWHEEL_VELOCITY, mFlywheelFalconMaster.getSelectedSensorVelocity()); //TODO needs to be converted from ticks per 100ms to RPS
        Robot.DATA.flywheel.set(CURRENT_FEEDER_VELOCITY, mFeeder.get());
        Robot.DATA.flywheel.set(CURRENT_HOOD_ANGLE, calcHoodAngle());
        Robot.DATA.flywheel.set(CURRENT_POTENTIOMETER_TURNS, mHoodPot.get() * 5);


//        if (Robot.DATA.limelight.isSet(ELimelightData.TV)) {
//            Robot.DATA.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, calcSpeedFromDistance(distanceFromTarget));
//            Robot.DATA.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, calcAngleFromDistance(distanceFromTarget));
//        } else {
//            Robot.DATA.flywheel.set(FLYWHEEL_DISTANCE_BASED_SPEED, 2000);
//            Robot.DATA.flywheel.set(SERVO_DISTANCE_BASED_ANGLE, 0);
//        }
    }

    @Override
    public void setOutputs(double pNow) {
        mFlywheelFalconMaster.set(ControlMode.Velocity, Robot.DATA.flywheel.get(TARGET_FLYWHEEL_VELOCITY));
        mFlywheelFalconFollower.set(ControlMode.Velocity, Robot.DATA.flywheel.get(TARGET_FLYWHEEL_VELOCITY));

        mFeeder.set(Robot.DATA.flywheel.get(TARGET_FEEDER_VELOCITY));

        setTurret();
        setHood();

        hoodAnglePID(0.005, 0, 0);
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.flywheel.set(TARGET_FLYWHEEL_VELOCITY, 0);
        Robot.DATA.flywheel.set(TARGET_FEEDER_VELOCITY, 0);
        Robot.DATA.flywheel.set(TARGET_HOOD_ANGLE, 0);
    }

    private boolean isPastVelocity(double pVelocity) {
        // TODO Convert ticks per 100 ms to rpm
        return mFlywheelFalconMaster.getSelectedSensorVelocity() >= pVelocity;
    }

    private double setTurret() {
        return 0.0; //neo smart motion, mimics elevator from 2019
    }

    private double calcHoodAngle() {
        return Robot.DATA.flywheel.get(CURRENT_POTENTIOMETER_TURNS) * kHoodConversionFactor;
    }

    private double setHood() {
        Robot.DATA.flywheel.get(TARGET_HOOD_ANGLE);
        return 0.0;
    }

//    private double calcSpeedFromDistance(Distance pDistance) {
//        //TODO figure out necessity
//        return 7.2E-3 * Math.pow(pDistance.inches(), 3)
//                - 0.209 * Math.pow(pDistance.inches(), 2)
//                + 6.31 * pDistance.inches()
//                + 227;
//    }
//
//    private double calcAngleFromDistance(Distance pDistance) {
//        //TODO tuning
//        return 5.2E-05 * Math.pow(pDistance.inches(), 4)
//                - 4.9E-03 * Math.pow(pDistance.inches(), 3)
//                + 0.157 * Math.pow(pDistance.inches(), 2)
//                + 2.94 * pDistance.inches()
//                + 68.2;
//    }

    private double hoodAnglePID(double pP, double pI, double pD) {
        // TODO Convert into an Angle Measurement
        double error = pP * (Robot.DATA.flywheel.get(TARGET_HOOD_ANGLE) - Robot.DATA.flywheel.get(CURRENT_HOOD_ANGLE));
        double velocity = pD * mHoodServo.getSpeed();
        mIntegral += pI * mHoodServo.getSpeed();
        return error + mIntegral + velocity;
    }

    private void potentiometerPID() {
        double output = Robot.DATA.flywheel.get(TARGET_HOOD_ANGLE) / 5;
    }
}