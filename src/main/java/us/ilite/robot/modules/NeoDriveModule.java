package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Units;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Pigeon;

import static us.ilite.common.types.drive.EDriveData.*;

public class NeoDriveModule extends Module {
    private CANSparkMax mRightMaster;
    private CANSparkMax mRightFollower;
    private CANSparkMax mLeftMaster;
    private CANSparkMax mLeftFollower;
    private RelativeEncoder mRightEncoder;
    private RelativeEncoder mLeftEncoder;
    private SparkMaxPIDController mRightCtrl;
    private SparkMaxPIDController mLeftCtrl;
    private PIDController mTurnToDegreePID;
    private Pigeon mGyro;

    // ========================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS
    // ========================================
    public static final double kGearboxRatio = (12.0 / 40.0) * (14.0 / 40.0);
    public static final double kWheelDiameterFeet = 3.9 / 12.0;
    public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;
    public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;

    // ========================================
    // DO NOT MODIFY THESE PID CONSTANTS
    // ========================================
    private static final int VELOCITY_PID_SLOT = 1;
    private static final int SMART_MOTION_PID_SLOT = 3;
    //TODO change the smart motion gains once we have tuned it
    public static ProfileGains kSmartMotionGains = new ProfileGains()
            .p(.00025)
            .f(0.00015)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .maxAccel(1000d)
            .slot(SMART_MOTION_PID_SLOT)
            .velocityConversion(kDriveNEOPositionFactor);
    public static ProfileGains kVelocityGains = new ProfileGains()
            .f(0.5)
            .p(0.00051968)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .slot(VELOCITY_PID_SLOT)
            .velocityConversion(kDriveNEOVelocityFactor);
    public static ProfileGains kTurnToProfileGains = new ProfileGains().p(0.0285);

    public NeoDriveModule() {
        mRightMaster = new CANSparkMax(Settings.HW.CAN.kDTMR2, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightFollower = new CANSparkMax(Settings.HW.CAN.kDTR4, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster = new CANSparkMax(Settings.HW.CAN.kDTML1, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftFollower = new CANSparkMax(Settings.HW.CAN.kDTL3, CANSparkMaxLowLevel.MotorType.kBrushless);
        mGyro = new Pigeon(Robot.CLOCK, Settings.HW.CAN.kDTGyro);

        mRightMaster.setInverted(true);
        mRightFollower.setInverted(true);

        mRightEncoder = mRightMaster.getEncoder();
        mLeftEncoder = mLeftMaster.getEncoder();

        mRightCtrl = mRightMaster.getPIDController();
        mLeftCtrl = mLeftMaster.getPIDController();
        mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);
        mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);

        mTurnToDegreePID = new PIDController(kTurnToProfileGains, -180, 180, Settings.kControlLoopPeriod);
        mTurnToDegreePID.setContinuous(true);
        mTurnToDegreePID.setOutputRange(-1, 1);
    }
    @Override
    public void readInputs() {
        db.drivetrain.set(L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(L_ACTUAL_POS_meters, Units.feet_to_meters(mLeftEncoder.getPosition() * kDriveNEOPositionFactor));
        db.drivetrain.set(L_ACTUAL_VEL_meters_s, Units.feet_to_meters(mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor));
        db.drivetrain.set(R_ACTUAL_POS_meters, Units.feet_to_meters(mRightEncoder.getPosition() * kDriveNEOPositionFactor));
        db.drivetrain.set(R_ACTUAL_VEL_meters_s, Units.feet_to_meters(mRightEncoder.getVelocity() * kDriveNEOVelocityFactor));
    }

    @Override
    public void setOutputs() {

    }
}
