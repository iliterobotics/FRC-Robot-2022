package us.ilite.robot.hardware;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import us.ilite.common.lib.control.ProfileGains;

public class HardwareUtils {

    public static void setGains(CANPIDController pNEO, ProfileGains pGains) {
        pNEO.setP(pGains.P, pGains.PROFILE_SLOT);
        pNEO.setI(pGains.I, pGains.PROFILE_SLOT);
        pNEO.setD(pGains.D, pGains.PROFILE_SLOT);
        pNEO.setFF(pGains.F, pGains.PROFILE_SLOT);
        pNEO.setSmartMotionAllowedClosedLoopError(pGains.TOLERANCE, pGains.PROFILE_SLOT);
        pNEO.setSmartMotionMaxAccel(pGains.MAX_ACCEL, pGains.PROFILE_SLOT);
        pNEO.setSmartMotionMaxVelocity(pGains.MAX_VELOCITY, pGains.PROFILE_SLOT);
    }

    public static void setGains(CANEncoder pEncoder, ProfileGains pGains) {
        pEncoder.setPositionConversionFactor(pGains.POSITION_CONVERSION_FACTOR);
        pEncoder.setVelocityConversionFactor(pGains.VELOCITY_CONVERSION_FACTOR);
    }

    public static void setGains(TalonFX pTalon, ProfileGains pGains) {
        pTalon.config_kF(pGains.PROFILE_SLOT, pGains.F);
        pTalon.config_kP(pGains.PROFILE_SLOT, pGains.P);
        pTalon.config_kI(pGains.PROFILE_SLOT, pGains.I);
        pTalon.config_kD(pGains.PROFILE_SLOT, pGains.D);
    }

    private HardwareUtils() {}
}
