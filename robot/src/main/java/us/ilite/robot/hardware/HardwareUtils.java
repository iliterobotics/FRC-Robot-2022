package us.ilite.robot.hardware;

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

    private HardwareUtils() {}
}
