package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EColorData;
import us.ilite.common.types.ELEDControlData;
import us.ilite.robot.Robot;

import static us.ilite.robot.modules.DJSpinnerModule.*;

public class LEDControl extends Module {

    private CANifier mLEDCan;
    private Timer mBlinkTimer;
    private Message mCurrentMessage;
    private Message mLastMessage;
    private LEDState mLEDState;

    public static class RGB {
        private int mR;
        private int mG;
        private int mB;

        public RGB(int pR, int pG, int pB) {
            // Value range for each color is 0-255, we'll enforce this with a module divide
            this.mR = pR % 256;
            this.mG = pG % 256;
            this.mB = pB % 256;
        }

        // getters for integer RGB
        public int getR() {
			return this.mR;
		}

        public int getG() {
			return this.mG;
		}

        public int getB() {
			return this.mB;
		}

        // getters for double RGB
        public double getRPercent() {
			return ((double) this.mR) / 256.0;
		}

        public double getGPercent() {
			return ((double) this.mG) / 256.0;
		}

        public double getBPercent() {
			return ((double) this.mB) / 256.0;
		}

    }


    public enum LEDColor {
        PURPLE( 255, 0, 200 ),
        RED( 255, 0, 0 ),
        LIGHT_BLUE( 0, 100, 220 ),
        WHITE( 255, 255, 255 ),
        GREEN( 0, 255, 0 ),
        YELLOW( 255, 255, 0 ),
        GREEN_HSV( 84, 255, 255 ),
        BLUE( 0, 0, 255 ),
        RED_HSV( 0, 255, 255 ),
        YELLOW_HSV( 20, 255, 255 ),
        PURPLE_HSV( 212, 255, 255 ),
        ORANGE( 255, 165, 0 ),
        NONE( 0, 0, 0 );

        private RGB rgb;

        LEDColor( int pR, int pG, int pB ) {
            this.rgb = new RGB(pR, pG, pB);
        }

        public RGB getColor() {
            return this.rgb;
        }
    }
    
    // pulse speed in milliseconds, 0 = on solid
    public enum Message {
        ON_BLUE( LEDColor.BLUE, false ),
        ON_RED( LEDColor.RED, false ),
        ON_GREEN( LEDColor.GREEN, false ),
        ON_YELLOW( LEDColor.YELLOW, false ),
        FINISHED_ON_BLUE( LEDColor.BLUE, true ),
        FINISHED_ON_RED( LEDColor.RED, true ),
        FINISHED_ON_GREEN( LEDColor.GREEN, true ),
        FINISHED_ON_YELLOW( LEDColor.YELLOW, true ),
        NONE(LEDColor.NONE, false),

        CURRENT_LIMITING( LEDColor.RED, false),
        VISION_TRACKING( LEDColor.GREEN, false);

        final LEDColor color;
        final int pulse; // milliseconds

        Message( LEDColor color, boolean isPulse ) {
            this.color = color;
            if ( isPulse == true ) {
                this.pulse = 300;
            }
            else {
                this.pulse = 0;
            }
        }

        static Message fromColorMatch(EColorMatch color, boolean isDone) {
            if (isDone) {
                if (color == EColorMatch.BLUE) {
                    return FINISHED_ON_BLUE;
                } else if (color == EColorMatch.RED) {
                    return FINISHED_ON_RED;
                } else if (color == EColorMatch.GREEN) {
                    return FINISHED_ON_GREEN;
                } else if (color == EColorMatch.YELLOW) {
                    return FINISHED_ON_YELLOW;
                } else {
                    return NONE;
                }
            } else {
                if (color == EColorMatch.BLUE) {
                    return ON_BLUE;
                } else if (color == EColorMatch.RED) {
                    return ON_RED;
                } else if (color == EColorMatch.GREEN) {
                    return ON_GREEN;
                } else if (color == EColorMatch.YELLOW) {
                    return ON_YELLOW;
                } else {
                    return NONE;
                }
            }
        }
    }
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


    public LEDControl(/*DJBoothPositionControl pDjBoothPositionControl, DJBoothRotationControl pDjBoothRotationControl */) {
//        this.mDjBoothPositionControl = pDjBoothPositionControl;
//        this.mDjBoothRotationControl = pDjBoothRotationControl;

        this.mCurrentMessage = Message.NONE;
        mLEDState = LEDState.ON;

        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
        mLEDCan = new CANifier(Settings.Hardware.CAN.kLEDControlCanifier);
    }


    public void modeInit(double pNow) {
        Robot.DATA.ledcontrol.set(ELEDControlData.LED_STATE , 1.0);
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }

    public void controlLED(Message m) {
        // Timer wants elapsed time in double seconds, pulse period specified in ms.
        double blinkPeriod = ((double) m.pulse) / 1000.0;

        if(m.pulse == 0) {
           mLEDState = LEDState.ON;
        }
        else if( this.mBlinkTimer.hasPeriodPassed(blinkPeriod) ) {
            mLEDState.isOn = !mLEDState.isOn;
            this.mBlinkTimer.stop();
            this.mBlinkTimer.reset();
            this.mBlinkTimer.start();
        }

        if(mLEDState.isOn()) {
            setLED(m.color);
        } else {
            turnOffLED();
        }

    }


    private void setLED(LEDColor color) {
        setLED(color.getColor());
    }

    // LED Channels: A = Green B = Red C = Blue
    private void setLED(RGB rgb) {
        mLEDCan.setLEDOutput(rgb.getRPercent(), CANifier.LEDChannel.LEDChannelB); // Red
        mLEDCan.setLEDOutput(rgb.getGPercent(), CANifier.LEDChannel.LEDChannelA); // Green
        mLEDCan.setLEDOutput(rgb.getBPercent(), CANifier.LEDChannel.LEDChannelC); // Blue
    }


    public void turnOffLED() {
        setLED(LEDColor.NONE);
    }


    @Override
    public void readInputs(double pNow) {
        //TODO decide what to put in here
        Robot.DATA.ledcontrol.set(ELEDControlData.LED_STATE , mLEDState.isOn() );
    }

    @Override
    public void setOutputs(double pNow) {
        Robot.DATA.ledcontrol.set(ELEDControlData.CURRENT_MESSAGE , Message.NONE);

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

    public void shutdown(double pNow) {
        // TODO Auto-generated method stub
        this.turnOffLED();
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }


}