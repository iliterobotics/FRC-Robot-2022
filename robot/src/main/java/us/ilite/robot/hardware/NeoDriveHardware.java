package us.ilite.robot.hardware;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Conversions;
import us.ilite.robot.modules.DriveMessage;

public class NeoDriveHardware implements IDriveHardware {

    private final ILog mLogger = Logger.createLog(NeoDriveHardware.class);
    private final double kGearRatio;

    private IMU mGyro;

    private final CANSparkMax mLeftMaster, mRightMaster, mLeftMiddle, mRightMiddle/*, mLeftRear, mRightRear*/;
    private ControlType mLeftControlMode, mRightControlMode;
    private CANSparkMax.IdleMode mLeftNeutralMode, mRightNeutralMode;

    public NeoDriveHardware(double pGearRatio) {
        kGearRatio = pGearRatio;
        mGyro = new Pigeon(Settings.Hardware.CAN.kPigeon);
        // mGyro = new NavX(SerialPort.Port.kMXP);

        mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kDriveLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMiddle = SparkMaxFactory.createFollowerSparkMax(Settings.Hardware.CAN.kDriveLeftMiddle, mLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);

        mRightMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kDriveRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMiddle = SparkMaxFactory.createFollowerSparkMax(Settings.Hardware.CAN.kDriveRightMiddle, mRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);

        configureMaster(mLeftMaster, true);
        configureMotor(mLeftMaster);
        configureMotor(mLeftMiddle);
//        configureMotor(mLeftRear);

        configureMaster(mRightMaster, false);
        configureMotor(mRightMaster);
        configureMotor(mRightMiddle);
//        configureMotor(mRightRear);

        mLeftMaster.setInverted(true);
        mLeftMiddle.setInverted(true);
//        mLeftRear.setInverted(true);

        mRightMaster.setInverted(false);
        mRightMiddle.setInverted(false);
//        mRightRear.setInverted(true);

        // Invert sensor readings by multiplying by 1 or -1
        mLeftMaster.getEncoder().setPositionConversionFactor(1.0 * kGearRatio);
        mLeftMaster.getEncoder().setVelocityConversionFactor(1.0 * kGearRatio);

        mRightMaster.getEncoder().setPositionConversionFactor(1.0 * kGearRatio);
        mRightMaster.getEncoder().setVelocityConversionFactor(1.0 * kGearRatio);


        reloadVelocityGains(mLeftMaster);
        reloadVelocityGains(mRightMaster);

    }

    @Override
    public void init() {
        zero();
        mLeftControlMode = mRightControlMode = ControlType.kDutyCycle;
        mLeftNeutralMode = mRightNeutralMode = CANSparkMax.IdleMode.kBrake;

        set(DriveMessage.kNeutral);
    }

    @Override
    public void zero() {
        mGyro.zeroAll();

        mLeftMaster.getEncoder().setPosition(0.0);
        mRightMaster.getEncoder().setPosition(0.0);

        // Bypass state machine in set() and configure directly
        configSparkForPercentOutput(mLeftMaster);
        configSparkForPercentOutput(mRightMaster);
        setNeutralMode(CANSparkMax.IdleMode.kBrake, mLeftMaster, mLeftMiddle/*, mLeftRear*/);
        setNeutralMode(CANSparkMax.IdleMode.kBrake, mRightMaster, mRightMiddle/*, mRightRear*/);

        mLeftMaster.set(0.0);
        mRightMaster.set(0.0);
    }

    public void set(DriveMessage pDriveMessage) {

        mLeftControlMode = configForControlMode(mLeftMaster, mLeftControlMode, pDriveMessage.getMode().kRevControlType);
        mRightControlMode = configForControlMode(mRightMaster, mRightControlMode, pDriveMessage.getMode().kRevControlType);

        mLeftNeutralMode = configForNeutralMode(mLeftNeutralMode, pDriveMessage.getNeutral().kRevIdleMode, mLeftMaster, mLeftMiddle/*, mLeftRear*/);
        mRightNeutralMode = configForNeutralMode(mRightNeutralMode, pDriveMessage.getNeutral().kRevIdleMode, mRightMaster, mRightMiddle/*, mRightRear*/);

        // TODO - update arbitrary FF with the ProfileGains FF
        mLeftMaster.getPIDController().setReference(pDriveMessage.getLeftOutput(), mLeftControlMode, 1, 0);
        mRightMaster.getPIDController().setReference(pDriveMessage.getRightOutput(), mRightControlMode, 1, 0);

    }

    /**
     * Allows external users to request that our control mode be pre-configured instead of configuring on the fly.
     * @param pControlMode
     */
    public void configureMode(ECommonControlMode pControlMode) {
        mLeftControlMode = configForControlMode(mLeftMaster, mLeftControlMode, pControlMode.kRevControlType);
        mRightControlMode = configForControlMode(mRightMaster, mRightControlMode, pControlMode.kRevControlType);
    }

    @Override
    public void setImu(IMU pImu) {
        mGyro = pImu;
    }

    public IMU getImu() {
        return mGyro;
    }

    private ControlType configForControlMode(CANSparkMax pSparkMax, ControlType pCurrentControlMode, ControlType pDesiredControlMode) {
        ControlType controlMode = pCurrentControlMode;

        if(pCurrentControlMode != pDesiredControlMode) {
            switch(pDesiredControlMode) {
                case kDutyCycle:
                    controlMode = ControlType.kDutyCycle;
                    configSparkForPercentOutput(pSparkMax);
                    break;
                case kSmartMotion:
                    controlMode = ControlType.kSmartMotion;
                    break;
                case kVelocity:
                    controlMode = ControlType.kVelocity;
                    break;
                default:
                    mLogger.error("Unimplemented control mode - defaulting to PercentOutput.");
                    controlMode = ControlType.kDutyCycle;
                    break;
            }
        }

        return controlMode;
    }

    private CANSparkMax.IdleMode configForNeutralMode(CANSparkMax.IdleMode pCurrentNeutralMode, CANSparkMax.IdleMode pDesiredNeutralMode, CANSparkMax... pSparkMaxes) {
        if(pCurrentNeutralMode != pDesiredNeutralMode) {
            setNeutralMode(pDesiredNeutralMode, pSparkMaxes);
        }

        return pDesiredNeutralMode;
    }

    private void setNeutralMode(CANSparkMax.IdleMode pNeutralMode, CANSparkMax ... pSparkMaxes) {
        for(CANSparkMax sparkMax : pSparkMaxes) {
            mLogger.info("Setting neutral mode to: ", pNeutralMode.name(), " for Talon ID ", sparkMax.getDeviceId());
            sparkMax.setIdleMode(pNeutralMode);
        }
    }

    private void configureMaster(CANSparkMax sparkMax, boolean pIsLeft) {
        // Velocity, temperature, voltage, and current according the REV docs
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 5);
        // Position according to REV docs
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);

        sparkMax.setSmartCurrentLimit(Settings.Drive.kCurrentLimitAmps);
        sparkMax.setSecondaryCurrentLimit(Settings.Drive.kCurrentLimitAmps);
        // Set a peak current limit duration??
    }

    private void configureMotor(CANSparkMax motorController) {
        /*
        TODO Disabled voltage comp for now because of:
        https://www.chiefdelphi.com/t/sparkmax-voltage-compensation/350540/5
         */
//        motorController.enableVoltageCompensation(12.0);
        // No velocity measurement filter
        motorController.setOpenLoopRampRate(Settings.Drive.kOpenLoopVoltageRampRate);
        motorController.setClosedLoopRampRate(Settings.Drive.kClosedLoopVoltageRampRate);
        // motorController.configNeutralDeadband(0.04, 0);
    }

    private void configSparkForPercentOutput(CANSparkMax pSparkMax) {
        // talon.configNeutralDeadband(0.04, 0);
    }

    private void reloadVelocityGains(CANSparkMax pSparkMax) {
        mLogger.info("Reloading gains for Talon ID ", pSparkMax.getDeviceId());

        CANPIDController sparkMaxPid = pSparkMax.getPIDController();
        HardwareUtils.setGains(sparkMaxPid, Settings.Drive.kVelocityPID);
    }

    private void configSparkForSmartMotion(CANSparkMax talon) {
        reloadVelocityGains(talon);
    }

    public double getLeftInches() {
        return Conversions.ticksToInches(mLeftMaster.getEncoder().getPosition());
    }

    public double getRightInches() {
        return Conversions.ticksToInches(mRightMaster.getEncoder().getPosition());
    }

    public double getLeftVelTicks() {
        return mLeftMaster.getEncoder().getVelocity();
    }

    public double getRightVelTicks() {
        return mRightMaster.getEncoder().getVelocity();
    }

    /**
     * TODO Not available with current API
     * @return
     */
    public double getLeftTarget() {
        return 0.0;
    }

    /**
     * TODO Not available with current API
     * @return
     */
    public double getRightTarget() {
        return 0.0;
    }

    public double getLeftVelInches() {
        return Conversions.ticksPer100msToRadiansPerSecond(getLeftVelTicks());
    }

    public double getRightVelInches() {
        return Conversions.ticksPer100msToRadiansPerSecond(getRightVelTicks());
    }

    @Override
    public double getLeftCurrent() {
        return mLeftMaster.getOutputCurrent();
    }

    @Override
    public double getRightCurrent() {
        return mRightMaster.getOutputCurrent();
    }

    @Override
    public double getLeftVoltage() {
        return mLeftMaster.getAppliedOutput() * 12.0;
    }

    @Override
    public double getRightVoltage() {
        return mRightMaster.getAppliedOutput() * 12.0;
    }

    @Override
    public boolean checkHardware() {

        // TODO Implement testing for VictorSPX
        // CheckerConfigBuilder checkerConfigBuilder = new CheckerConfigBuilder();
        // checkerConfigBuilder.setCurrentFloor(2);
        // checkerConfigBuilder.setCurrentEpsilon(2.0);
        // checkerConfigBuilder.setRPMFloor(1500);
        // checkerConfigBuilder.setRPMEpsilon(250);
        // checkerConfigBuilder.setRPMSupplier(()->mLeftMaster.getSelectedSensorVelocity(0));

        // boolean leftSide = TalonSRXChecker.CheckTalons(Drive.class,
        //         Arrays.asList(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster),
        //             new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftRear)),
        //         checkerConfigBuilder.build());

        // checkerConfigBuilder.setRPMSupplier(()->mRightMaster.getSelectedSensorVelocity(0));

        // boolean rightSide = TalonSRXChecker.CheckTalons(Drive.class,
        //         Arrays.asList(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster),
        //                 new TalonSRXChecker.TalonSRXConfig("right_slave", mRightRear)),
        //         checkerConfigBuilder.build());
        // return leftSide && rightSide;
        return true;
    }

}
