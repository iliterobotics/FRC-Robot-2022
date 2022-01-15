package us.ilite.robot.modules;

import com.revrobotics.*;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.SparkMaxFactory;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static us.ilite.common.types.EHangerModuleData.*;

public class HangerModule extends Module {

    private CANSparkMax mLeftHanger;
    private CANSparkMax mRightHanger;
    private CANPIDController mLeftHangerPID;
    private CANPIDController mRightHangerPID;

    private CANEncoder mLeftHangerEncoder;
    private CANEncoder mRightHangerEncoder;

    public static double kMaxRPM = 2000.0;
    private static final int VELOCITY_PID_SLOT = 0;
    public static ProfileGains kHangerVelocityGains = new ProfileGains()
            .f(0.0002)
            .p(0.00015)
            .maxVelocity(kMaxRPM)
            .maxAccel(kMaxRPM*1.5)
            .slot(VELOCITY_PID_SLOT);

    private int kHangerWarnCurrentLimitThreshold = 100;

    public HangerModule(){

        mLeftHanger = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kMAXHanger1_left,kBrushless);
        mRightHanger = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kMAXHanger2_right, kBrushless);

        mLeftHangerPID = new CANPIDController(mLeftHanger);
        mRightHangerPID = new CANPIDController(mRightHanger);

        mLeftHanger.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightHanger.setIdleMode(CANSparkMax.IdleMode.kBrake);

        HardwareUtils.setGains(mLeftHangerPID, kHangerVelocityGains);
        HardwareUtils.setGains(mRightHangerPID, kHangerVelocityGains);

        mLeftHanger.setInverted(true);
        mRightHanger.setInverted(false);

        mLeftHanger.setSmartCurrentLimit(kHangerWarnCurrentLimitThreshold);
        mRightHanger.setSmartCurrentLimit(kHangerWarnCurrentLimitThreshold);

        mLeftHangerEncoder = mLeftHanger.getEncoder();
        mRightHangerEncoder = mRightHanger.getEncoder();

        mLeftHanger.burnFlash();
        mRightHanger.burnFlash();

        mLeftHangerEncoder.setPosition(0);
        mRightHangerEncoder.setPosition(0);
    }

    @Override
    public void readInputs() {
        db.hanger.set(L_OUTPUT_CURRENT, mLeftHanger.getOutputCurrent());
        db.hanger.set(R_OUTPUT_CURRENT, mRightHanger.getOutputCurrent());
        db.hanger.set(L_VEL_rpm, mLeftHangerEncoder.getVelocity());
        db.hanger.set(R_VEL_rpm, mRightHangerEncoder.getVelocity());
        db.hanger.set(L_POSITION_rot, mLeftHangerEncoder.getPosition());
        db.hanger.set(R_POSITION_rot, mRightHangerEncoder.getPosition());
    }

    @Override
    public void modeInit(EMatchMode pMode){


    }

    @Override
    public void setOutputs() {
        double desiredPct = db.hanger.safeGet(SET_pct, 0.0);
        mLeftHangerPID.setReference(desiredPct * kMaxRPM, ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
        mRightHangerPID.setReference(desiredPct * kMaxRPM, ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
    }
}
