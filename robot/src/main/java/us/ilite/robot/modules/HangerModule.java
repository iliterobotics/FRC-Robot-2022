package us.ilite.robot.modules;

import com.revrobotics.*;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;

public class HangerModule extends Module {

    private CANSparkMax mHangerNeoMaster;
    private CANSparkMax mHangerNeoFollower;
    protected EHangerState mHangerState;
    private CANPIDController mHangerPIDMaster;
    private CANPIDController mHangerPIDFollower;
    private double mStartingPosOne;
    private double mStartingPosTwo;
    private boolean mSetSetpoint = false;

    private CANEncoder mHangerEncoderOne;
    private CANEncoder mHangerEncoderTwo;

    public static double kMaxRPM = 2000.0;
    private static final int VELOCITY_PID_SLOT = 0;
    private static final int POSITION_PID_SLOT = 1;
    public static ProfileGains kHangerVelocityGains = new ProfileGains()
            .f(0.0002)
            .p(0.00015)
            // Enforce a maximum allowed speed, system-wide. DO NOT undo kMaxAllowedVelocityMultiplier without checking with a mentor first.
            .maxVelocity(kMaxRPM)
            .maxAccel(kMaxRPM*1.5)
            .slot(VELOCITY_PID_SLOT);

    public static ProfileGains kHangerPositionGains = new ProfileGains()
//			.p(5.0e-4)
            .maxVelocity(kMaxRPM)
            .f(0.00015)
            .maxAccel(kMaxRPM * 1.5)
            .slot(POSITION_PID_SLOT);

    private int kHangerWarnCurrentLimitThreshold = 100;

    public HangerModule(){

        mHangerNeoMaster = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID1 ,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mHangerNeoFollower = SparkMaxFactory.createDefaultSparkMax(Settings.Hardware.CAN.kHangerNeoID2 , CANSparkMaxLowLevel.MotorType.kBrushless);


        mHangerPIDMaster = new CANPIDController(mHangerNeoMaster);
        mHangerPIDFollower = new CANPIDController(mHangerNeoFollower);
        HardwareUtils.setGains(mHangerPIDMaster, kHangerVelocityGains);
        HardwareUtils.setGains(mHangerPIDFollower, kHangerVelocityGains);
        HardwareUtils.setGains(mHangerPIDMaster, kHangerPositionGains);
        HardwareUtils.setGains(mHangerPIDFollower, kHangerPositionGains);

        mHangerNeoMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mHangerNeoFollower.setInverted(false);
        mHangerNeoMaster.setInverted(true);


        mHangerNeoMaster.burnFlash();
        mHangerNeoFollower.burnFlash();

        mHangerEncoderOne = mHangerNeoMaster.getEncoder();
        mHangerEncoderTwo = mHangerNeoFollower.getEncoder();

        mHangerNeoMaster.setSmartCurrentLimit(kHangerWarnCurrentLimitThreshold);
        mHangerNeoFollower.setSmartCurrentLimit(kHangerWarnCurrentLimitThreshold);

        zeroTheEncoders();
    }
    public enum EHangerState {
        HANGING(1.0),
        BRAKE (0.0),
        REVERSE(-1.0);

        private double position;
        EHangerState(double position){
            this.position = position;
        }
        private double getPower(){
            return this.position;
        }

    }

    @Override
    public void readInputs(double pNow) {
        db.hanger.set(EHangerModuleData.OUTPUT_CURRENT , mHangerNeoMaster.getOutputCurrent());
        db.hanger.set(EHangerModuleData.POS_ONE_ROT, mHangerEncoderOne.getPosition());
        db.hanger.set(EHangerModuleData.POS_TWO_ROT , mHangerEncoderTwo.getPosition());
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow){


    }

    @Override
    public void setOutputs(double pNow) {
//        mHangerNeoFollower.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_PCT));
//        mHangerNeoMaster.set(Robot.DATA.hanger.get(EHangerModuleData.DESIRED_PCT));
        Enums.EHangerControlState state = db.hanger.safeGet(EHangerModuleData.STATE, Enums.EHangerControlState.HOLD, Enums.EHangerControlState.class);

        switch (state) {
            case MOVE:
                double desiredPct = db.hanger.safeGet(EHangerModuleData.DESIRED_PCT, 0.0);
                mHangerPIDMaster.setReference(desiredPct * kMaxRPM, ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mHangerPIDFollower.setReference(desiredPct * kMaxRPM, ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
            case HOLD:
                if (!mSetSetpoint) {
                    mStartingPosOne = mHangerEncoderOne.getPosition();
                    mStartingPosTwo = mHangerEncoderTwo.getPosition();
                    mSetSetpoint = true;
                }
                mHangerPIDMaster.setReference(1, ControlType.kPosition, POSITION_PID_SLOT, 0);
                mHangerPIDFollower.setReference(1, ControlType.kPosition, POSITION_PID_SLOT, 0);
        }
    }

    public EHangerState returnHangerState() {
        return this.mHangerState;
    }
    public void putDesiredHangerState(EHangerState desiredState){
        mHangerState = desiredState;
    }
    public boolean isCurrentLimiting(){
        return mHangerNeoMaster.getOutputCurrent() >= kHangerWarnCurrentLimitThreshold ||
                mHangerNeoFollower.getOutputCurrent() >= kHangerWarnCurrentLimitThreshold;
    }

    public void zeroTheEncoders(){
        mHangerEncoderOne.setPosition(0);
        mHangerEncoderTwo.setPosition(0);
    }

}
