package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EColorData;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.LEDControlUtils.*;

import static us.ilite.robot.modules.DJSpinnerModule.*;

public class LEDControl extends Module {

    private CANifier mLEDCan;
    private Timer mBlinkTimer;
    private Message mCurrentMessage;
    private Message mLastMessage;
    private LEDState mLEDState;

    public enum LEDState {
        ON(true),
        OFF(false);
        boolean isOn;
        LEDState(boolean isOn){
            this.isOn = isOn;
        }
        public boolean isOn(){
            return isOn;
        }

    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {
        Robot.DATA.ledcontrol.set(ELEDControlData.LED_STATE , 1.0);
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }

    @Override
    public void readInputs(double pNow) {
        db.ledcontrol.set(ELEDControlData.LED_STATE , mLEDState.isOn() );
    }

    @Override
    public void setOutputs(double pNow) {
        EColorMatch color = EColorMatch.values()[(int) db.color.get(EColorData.SENSED_COLOR)];
        boolean isDone = (EColorWheelState.valueOf(db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE)) == EColorWheelState.OFF);
        mCurrentMessage = Message.fromColorMatch(color, isDone);

        // Did the message change?
        if ( mLastMessage != this.mCurrentMessage ) {
            // The message changed, reset the timer and on state
            mLEDState = LEDState.ON;
            this.mBlinkTimer.stop();
            this.mBlinkTimer.reset();
            this.mBlinkTimer.start();
        }

        controlLED(Message.fromColorMatch(color, isDone));
        mLastMessage = mCurrentMessage;
    }

    public LEDControl() {
        this.mCurrentMessage = Message.NONE;
        mLEDState = LEDState.ON;

        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
        mLEDCan = new CANifier(Settings.Hardware.CAN.kLEDControlCanifier);
    }

    public void controlLED(Message m) {
        // Timer wants elapsed time in double seconds, pulse period specified in ms.
        double blinkPeriod = ((double) m.pulse) / 1000.0;

        if(m.pulse == 0) {
            mLEDState = LEDState.ON;
        } else if( this.mBlinkTimer.hasPeriodPassed(blinkPeriod) ) {
            mLEDState.isOn = !mLEDState.isOn;
            this.mBlinkTimer.stop();
            this.mBlinkTimer.reset();
            this.mBlinkTimer.start();
        }

        if(mLEDState.isOn()) {
            setLED(m.color);
        } else {
            defaultLED();
        }
    }

    public void defaultLED() {
        setLED(LEDColor.DEFAULT);
    }

    private void setLED(LEDColor color) {
        setLED(color.getColor());
    }

    private void setLED(RGB rgb) {
        mLEDCan.setLEDOutput(rgb.getRPercent(), CANifier.LEDChannel.LEDChannelB); // Red
        mLEDCan.setLEDOutput(rgb.getGPercent(), CANifier.LEDChannel.LEDChannelA); // Green
        mLEDCan.setLEDOutput(rgb.getBPercent(), CANifier.LEDChannel.LEDChannelC); // Blue
    }

    public void shutdown(double pNow) {
        // TODO Auto-generated method stub
        this.defaultLED();
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }
}