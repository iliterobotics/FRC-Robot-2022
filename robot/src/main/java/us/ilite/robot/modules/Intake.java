package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.CANSparkMax;
import us.ilite.common.Data;

public class Intake extends Module {

    private CANSparkMax mIntakeMotor;
    private TalonSRX mTalonOne;
    private TalonSRX mTalonTwo;
    private TalonSRX mTalonThree;

    private Data mData;

    private ILog mLog = Logger.createLog(this.getClass());


    public enum EIntakeState{
        INTAKE,
        STOP,
        REVERSE;
    }

    public Intake( Data pData ) {
        this.mData = pData;
    }

    @Override
    public void modeInit(double pNow) {

    }

    @Override
    public void periodicInput(double pNow) {

    }

    @Override
    public void update(double pNow) {

    }

    @Override
    public void shutdown(double pNow) {

    }
}
