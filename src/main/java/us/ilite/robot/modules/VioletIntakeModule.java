package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.robot.Enums.*;

import us.ilite.robot.Enums;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.hardware.TalonSRXFactory;

public class VioletIntakeModule extends Module {

    // Serializer Motors
    private TalonSRX mConveyorMotorHorizontal;
    private TalonSRX mConveyorMotorVertical;

    //Arm
    private CANSparkMax mIntakePivot;
    private CANSparkMax mIntakeRoller;
    private RelativeEncoder mIntakePivotEncoder;
    private RelativeEncoder mIntakeRollerEncoder;
    private double kArmMinDegrees = 0;
    private double kArmMaxDegrees = 90;

    //    Beam Breakers
    private DigitalBeamSensor mEntryBeam;
    private DigitalBeamSensor mSecondaryBeam;
    private DigitalBeamSensor mExitBeam;

    //Constants
    private static final double kIntakeRollerRatio = 12.0 / 30.0 * 22.0 / 36.0;
    private static final double kIntakeRollerDiameterFeet = 1.5 / 12.0;
    private static final double kIntakeRollerSpeedConversion = kIntakeRollerRatio * kIntakeRollerDiameterFeet * Math.PI / 60.0;
    private static final double kPivotGearRatio = 1.0/100.0 * 16.0 / 36.0;
    private static final double kPivotConversion = kPivotGearRatio * 360.0;
    // RPM to degrees / second
    private static final double kPivotVelocityConversion = kPivotConversion / 60.0;
    private static final double kPivotAngleConversionFactor = 0.0;
    private static final double kMaxIntakePivotVelocityDeg_s = 10.0;
    private static final ProfileGains mIntakePivotDownGains = new ProfileGains()
            .p(0.000025)
//            .i(0.000000001)
//            .d(0.000001)
//            .maxVelocity(0.5)
            .maxVelocity(convertDegreesPerSecondToRPM(kMaxIntakePivotVelocityDeg_s)/5500)
            ;
    private static final ProfileGains mIntakeRollerGains = new ProfileGains()
            .f(0.00015)
//            .p(0.0001)
            .maxAccel(9000d)
            .maxVelocity(11000d)
            ;

    private PIDController mIntakePivotPID;
    private us.ilite.common.lib.control.PIDController mIntakeRollerPID;

    // These are PRODUCTION BOT values
    // TODO - calbirate
    private static final double kPivotAbsoluteMin = -0.32;
    private static final double kPivotAbsoluteMax = 0.25;
    private static final ProfileGains mIntakePivotUpGains = mIntakePivotDownGains;

    private ILog mLog = Logger.createLog(this.getClass());

    public VioletIntakeModule() {
        mIntakeRoller = SparkMaxFactory.createDefaultSparkMax( Settings.HW.CAN.kMAXIntakeRollerId, CANSparkMaxLowLevel.MotorType.kBrushless );
        mIntakeRoller.setInverted(true);
        mIntakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mIntakeRoller.setSmartCurrentLimit(40);
        mIntakeRollerEncoder = mIntakeRoller.getEncoder();

        mConveyorMotorHorizontal = TalonSRXFactory.createDefaultTalon( Settings.HW.CAN.kTalonintakeSerializer);
        mConveyorMotorVertical = TalonSRXFactory.createDefaultTalon( Settings.HW.CAN.kTalonVerticalID );
        mConveyorMotorVertical.setInverted(true);

        mIntakePivot = SparkMaxFactory.createDefaultSparkMax( Settings.HW.CAN.kMAXIntakeArm, CANSparkMaxLowLevel.MotorType.kBrushless);
        mIntakePivot.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mIntakePivot.setSmartCurrentLimit(40);

        double debounceTime_s = 0.1;
        mEntryBeam = new DigitalBeamSensor( Settings.HW.DIO.kEntryBeamChannel, debounceTime_s);
        mSecondaryBeam = new DigitalBeamSensor( Settings.HW.DIO.kSecondaryBeamChannel, debounceTime_s);
        mExitBeam = new DigitalBeamSensor( Settings.HW.DIO.kExitBeamChannel, debounceTime_s);

        mIntakePivotEncoder = mIntakePivot.getEncoder();

        mIntakePivotPID = new PIDController(0.008, 0.0000025, 0);

        mIntakeRollerPID = new us.ilite.common.lib.control.PIDController(mIntakeRollerGains, -18, 18, clock.dt());
        mIntakeRoller.burnFlash();
        mIntakePivot.burnFlash();
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        mIntakePivotEncoder.setPosition(0.0);
        mIntakeRollerEncoder.setPosition(0.0);

        mIntakeRollerPID.setOutputRange(-0.5, 0.5);
    }

    @Override
    public void readInputs() {
        db.intake.set(INTAKE_VEL_ft_s, mIntakeRollerEncoder.getVelocity() * kIntakeRollerSpeedConversion);
        db.intake.set(ARM_ANGLE_deg, mIntakePivotEncoder.getPosition() * kPivotConversion);
        db.intake.set(ENTRY_BEAM, mEntryBeam.isBroken());
        db.intake.set(H_BEAM, mSecondaryBeam.isBroken());
        db.intake.set(EXIT_BEAM, mExitBeam.isBroken());
    }

    @Override
    public void setOutputs() {
        setPivotArm();
        setSerializer();
        mIntakeRollerPID.setSetpoint(10);
        double velocity = mIntakeRollerPID.calculate(db.intake.get(INTAKE_VEL_ft_s), clock.getCurrentTimeInMillis());
        velocity = (velocity == Double.NaN) ? 0.5 : velocity;
        SmartDashboard.putNumber("Roller Velocity", velocity);
        mIntakeRoller.set(db.intake.get(SET_INTAKE_VEL_ft_s) / 18);
    }

    private void setPivotArm() {
        EArmState mode = db.intake.get(INTAKE_STATE, EArmState.class);
        double desiredAngle = 0;
        SmartDashboard.putNumber("Arm pid value: ", desiredAngle);
        if (mode == null) return;
        switch(mode){
            case STOW:
                desiredAngle = mIntakePivotPID.calculate(db.intake.get(ARM_ANGLE_deg), kArmMinDegrees);
                break;
            case OUT:
                desiredAngle = mIntakePivotPID.calculate(db.intake.get(ARM_ANGLE_deg), kArmMaxDegrees);
                break;
            case NONE:
                db.intake.set(INTAKE_STATE, EArmState.STOW);
                mode = db.intake.get(INTAKE_STATE, EArmState.class);
                break;
        }

        mIntakePivot.set(desiredAngle);
    }

    private void setSerializer() {
        mConveyorMotorHorizontal.set(ControlMode.PercentOutput, db.intake.get(SET_H_pct));
        mConveyorMotorVertical.set(ControlMode.PercentOutput, db.intake.get(SET_V_pct));
    }

    private static double convertDegreesPerSecondToRPM(double dps) {
        return 60.0*dps/(kPivotGearRatio*17.0/12.0);
    }
}