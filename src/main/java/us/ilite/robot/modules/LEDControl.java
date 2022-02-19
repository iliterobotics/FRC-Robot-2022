package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.config.Settings;

import static us.ilite.robot.Enums.*;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EMatchMode;

public class LEDControl extends Module {

    private CANifier mLEDCan;
    private Timer mBlinkTimer;
    private LEDColorMode mColorMode = LEDColorMode.DEFAULT;
    private final double kBlinkTime = 0.5;

    public LEDControl() {
        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
        mLEDCan = new CANifier(22);
    }

    public void modeInit(EMatchMode pMode) {
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }

    public void readInputs() {
        db.ledcontrol.set(ELEDControlData.LED_STATE , isBlinking(ELEDControlData.LED_STATE));
        db.ledcontrol.set(ELEDControlData.DESIRED_R, db.ledcontrol.get(ELEDControlData.DESIRED_R));
        db.ledcontrol.set(ELEDControlData.DESIRED_G, db.ledcontrol.get(ELEDControlData.DESIRED_G));
        db.ledcontrol.set(ELEDControlData.DESIRED_B, db.ledcontrol.get(ELEDControlData.DESIRED_B));
    }

    public boolean isBlinking(ELEDControlData pData) {
        //return true if leds are blinking
        return db.ledcontrol.get(ELEDControlData.LED_STATE) == 1d;
    }

    @Override
    public void setOutputs() {
        if (db.ledcontrol.get(ELEDControlData.LED_STATE) == 1d) {
            //start timer and turn off if mBlinkTimer < half of cycle time, and on if mBlinkTimer > half of cycle time
            mBlinkTimer.start();
            if(mBlinkTimer.hasElapsed(kBlinkTime / 2)) {
                mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_R)/ 255, CANifier.LEDChannel.LEDChannelA);
                mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_G)/ 255, CANifier.LEDChannel.LEDChannelB);
                mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_B)/ 255, CANifier.LEDChannel.LEDChannelC);
                if(mBlinkTimer.hasElapsed(kBlinkTime)) {
                    mBlinkTimer.reset();
                }
            }
            else {
                mLEDCan.setLEDOutput(0d, CANifier.LEDChannel.LEDChannelA);
                mLEDCan.setLEDOutput(0d, CANifier.LEDChannel.LEDChannelB);
                mLEDCan.setLEDOutput(0d, CANifier.LEDChannel.LEDChannelC);
            }
        }
        else {
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_R)/ 255, CANifier.LEDChannel.LEDChannelA);
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_G)/ 255, CANifier.LEDChannel.LEDChannelB);
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_B)/ 255, CANifier.LEDChannel.LEDChannelC);
        }
    }
}