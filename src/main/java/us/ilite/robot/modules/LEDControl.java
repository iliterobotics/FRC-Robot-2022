package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.config.Settings;

import static us.ilite.robot.Enums.*;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EMatchMode;


public class LEDControl extends Module {

    private CANifier mLEDCan;
    private Timer mBlinkTimer;
    private LEDColorMode mColor = LEDColorMode.DEFAULT;

//    public enum LEDColor {
//        PURPLE( 255, 0, 200 ),
//        RED( 255, 0, 0 ),
//        LIGHT_BLUE( 0, 100, 220 ),
//        WHITE( 255, 255, 255 ),
//        GREEN( 0, 255, 0 ),
//        YELLOW( 255, 255, 0 ),
//        GREEN_HSV( 84, 255, 255 ),
//        BLUE( 0, 0, 255 ),
//        RED_HSV( 0, 255, 255 ),
//        YELLOW_HSV( 20, 255, 255 ),
//        PURPLE_HSV( 212, 255, 255 ),
//        ORANGE( 255, 165, 0 ),
//        DEFAULT( 0, 0, 0 );
//    }

    public void modeInit(EMatchMode pMode) {
      //  Robot.DATA.ledcontrol.set(LEDControlMode.LED_STATE , 1.0);

//        this.mBlinkTimer.stop();
//        this.mBlinkTimer.reset();
    }

    public void readInputs() {
        db.ledcontrol.set(ELEDControlData.LED_STATE , isOn(ELEDControlData.LED_STATE));
    }

    public boolean isOn(ELEDControlData pData) {
        if(db.ledcontrol.get(ELEDControlData.LED_STATE) == 1.0 ) {
            return true;
        }
        return false;
    }

    @Override
    public void setOutputs() {
        //trying to set the LED output to a color
        if (db.ledcontrol.get(ELEDControlData.LED_STATE) == 1.0) {
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_R)/ 255, CANifier.LEDChannel.LEDChannelB);
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_G)/ 255, CANifier.LEDChannel.LEDChannelA);
            mLEDCan.setLEDOutput(db.ledcontrol.get(ELEDControlData.DESIRED_B)/ 255, CANifier.LEDChannel.LEDChannelC);
        }
        else {
            mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
            mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
            mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
        }

    }

    public LEDControl() {
        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
        mLEDCan = new CANifier(Settings.HW.CAN.kLEDCanifier);
    }

//    public void defaultLED() {
//        setLED(LEDControlMode.RGB.getColor());
//    }

    private void setLED(LEDColorMode color) {
        mColor = color;
    }
}