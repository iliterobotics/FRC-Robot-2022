package us.ilite.robot.hardware;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;

public enum ECommonNeutralMode {
    NULL(null, null),
    BRAKE(NeutralMode.Brake, CANSparkMax.IdleMode.kBrake),
    COAST(NeutralMode.Coast, CANSparkMax.IdleMode.kCoast),
    HOLD_POSITION(NeutralMode.Brake, CANSparkMax.IdleMode.kBrake);

    public final NeutralMode kCtreNeutralMode;
    public final CANSparkMax.IdleMode kRevIdleMode;

    ECommonNeutralMode(NeutralMode pKCtreNeutralMode, CANSparkMax.IdleMode pKRevIdleMode) {
        kCtreNeutralMode = pKCtreNeutralMode;
        kRevIdleMode = pKRevIdleMode;
    }

}
