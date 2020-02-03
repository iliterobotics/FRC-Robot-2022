package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.types.EColorData;
import static us.ilite.robot.modules.DJSpinnerModule.*;

public class LEDControl extends Module {

    private CANifier mCanifier;
    private Timer mBlinkTimer;
    private boolean mLedOn;
    private Message mCurrentMessage;
//    private DJBoothPositionControl mDjBoothPositionControl;
//    private DJBoothRotationControl mDjBoothRotationControl;
    
//    private final Drive mDrive;
//    private final Elevator mElevator;
//    private final PneumaticIntake mPneumaticIntake;
//    private final CargoSpit mCargoSpit;
//    private final HatchFlower mHatchFlower;
//    private final FourBar mFourBar;
//    private final Limelight mLimelight;
//    private final Data mData;


    public static class RGB {
        private int mR;
        private int mG;
        private int mB;

        public RGB(int pR, int pG, int pB) {
            // Value range for each color is 0-255, we'll enforce this with a modulo divide
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
//        HAS_HATCH( LEDColor.YELLOW, 0 ),
//        HAS_CARGO( LEDColor.ORANGE, 0 ),
//        CURRENT_LIMITING( LEDColor.RED, 300 ),
//        VISION_TRACKING( LEDColor.GREEN, 0 ),
//        KICKING_HATCH( LEDColor.BLUE, 0 ),
//        SPITTING_CARGO( LEDColor.WHITE, 0 ),
//        NONE( LEDColor.NONE, 0 );

        ON_BLUE( LEDColor.BLUE, false ),
        ON_RED( LEDColor.RED, false ),
        ON_GREEN( LEDColor.GREEN, false ),
        ON_YELLOW( LEDColor.YELLOW, false ),
        FINISHED_ON_BLUE( LEDColor.BLUE, true ),
        FINISHED_ON_RED( LEDColor.RED, true ),
        FINISHED_ON_GREEN( LEDColor.GREEN, true ),
        FINISHED_ON_YELLOW( LEDColor.YELLOW, true ),
        NONE(LEDColor.NONE, false);

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
    }


    public LEDControl(/*DJBoothPositionControl pDjBoothPositionControl, DJBoothRotationControl pDjBoothRotationControl */) {
//        this.mDjBoothPositionControl = pDjBoothPositionControl;
//        this.mDjBoothRotationControl = pDjBoothRotationControl;

        this.mCurrentMessage = Message.NONE;
        this.mLedOn = true;

        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
    }


    public void modeInit(double pNow) {
        this.turnOffLED();
        this.mCurrentMessage = Message.NONE;
        this.mLedOn = true;
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }


    /**
     * Updates LED strip based on mechanism states. We check mechanisms in order of lowest to highest priority.
     */
    public void update(double pNow) {
        Message lastMsg = this.mCurrentMessage;
        this.mCurrentMessage = Message.NONE;
        
//        if(mCargoSpit.isCurrentLimiting()) mCurrentMessage = Message.CURRENT_LIMITING;
//        if(mElevator.isCurrentLimiting()) mCurrentMessage = Message.CURRENT_LIMITING;
//        if(mDrive.isCurrentLimiting()) mCurrentMessage = Message.CURRENT_LIMITING;
//        if(mFourBar.isCurrentLimiting()) mCurrentMessage = Message.CURRENT_LIMITING;
//
//        if(mCargoSpit.hasCargo()) mCurrentMessage = Message.HAS_CARGO;
//        if(mHatchFlower.hasHatch()) mCurrentMessage = Message.HAS_HATCH;
//
//        if(mCargoSpit.isOuttaking()) mCurrentMessage = Message.SPITTING_CARGO;
//        if(mHatchFlower.shouldBackUp()) mCurrentMessage = Message.KICKING_HATCH;
//
//        if(mLimelight.getTracking() != ETrackingType.NONE) mCurrentMessage = Message.VISION_TRACKING;
//

        EColorMatch color = EColorMatch.values()[(int)(double)db.color.get(EColorData.SENSED_COLOR)];
        boolean isDone = (EColorWheelState.valueOf(db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE)) == EColorWheelState.OFF);


        if (color == EColorMatch.BLUE && !isDone) {
            mCurrentMessage = Message.ON_BLUE;
        } else if (color == EColorMatch.RED && !isDone) {
            mCurrentMessage = Message.ON_RED;
        } else if (color == EColorMatch.GREEN && !isDone) {
            mCurrentMessage = Message.ON_GREEN;
        } else if (color == EColorMatch.YELLOW && !isDone) {
            mCurrentMessage = Message.ON_YELLOW;
        }
        else if (color == EColorMatch.BLUE && isDone) {
            mCurrentMessage = Message.FINISHED_ON_BLUE;
        } else if (color == EColorMatch.RED && isDone) {
            mCurrentMessage = Message.FINISHED_ON_RED;
        } else if (color == EColorMatch.GREEN && isDone) {
            mCurrentMessage = Message.FINISHED_ON_GREEN;
        } else if (color == EColorMatch.YELLOW && isDone) {
            mCurrentMessage = Message.FINISHED_ON_YELLOW;
        }

//        if ( Math.abs( red - LEDColor.RED.getColor().getR() ) <= 10  &&
//                Math.abs( green - LEDColor.RED.getColor().getG() ) <= 10  &&
//                Math.abs(blue - LEDColor.RED.getColor().getB()) <= 10 ) {
//            mCurrentMessage = Message.ON_RED;
//        }
//        else if ( Math.abs( red - LEDColor.BLUE.getColor().getR() ) <= 10  &&
//                Math.abs( green - LEDColor.BLUE.getColor().getG() ) <= 10  &&
//                Math.abs(blue - LEDColor.BLUE.getColor().getB()) <= 10) {
//            mCurrentMessage = Message.ON_BLUE;
//        }
//        else if ( Math.abs( red - LEDColor.GREEN.getColor().getR() ) <= 10  &&
//                Math.abs( green - LEDColor.GREEN.getColor().getG() ) <= 10  &&
//                Math.abs(blue - LEDColor.GREEN.getColor().getB()) <= 10) {
//            mCurrentMessage = Message.ON_GREEN;
//        }
//        else if ( Math.abs( red - LEDColor.YELLOW.getColor().getR() ) <= 10  &&
//                Math.abs( green - LEDColor.YELLOW.getColor().getG() ) <= 10  &&
//                Math.abs(blue - LEDColor.YELLOW.getColor().getB()) <= 10) {
//            mCurrentMessage = Message.ON_YELLOW;
//        }

        // Did the message change?
        if ( lastMsg != this.mCurrentMessage ) {
            // The message changed, reset the timer and on state
            this.mLedOn = true;
            this.mBlinkTimer.stop();
            this.mBlinkTimer.reset();
            this.mBlinkTimer.start();
        }

        controlLED(mCurrentMessage);
    }

    public void controlLED(Message m)
    {
        // Timer wants elapsed time in double seconds, pulse period specified in ms.
        double blinkPeriod = ((double) m.pulse) / 1000.0;

        if(m.pulse == 0)
        {
            mLedOn = true;
        }
        else if( this.mBlinkTimer.hasPeriodPassed(blinkPeriod) ) {
            mLedOn = !mLedOn;
            this.mBlinkTimer.stop();
            this.mBlinkTimer.reset();
            this.mBlinkTimer.start();
        }

        if(mLedOn) {
            setLED(m.color);
        } else {
            turnOffLED();
        }

    }


    private void setLED(LEDColor color) {
        setLED(color.getColor());
    }

    // LED Channels: A = Green B = Red C = Blue
    private void setLED(RGB rgb)
    {
        mCanifier.setLEDOutput(rgb.getRPercent(), CANifier.LEDChannel.LEDChannelB); // Red
        mCanifier.setLEDOutput(rgb.getGPercent(), CANifier.LEDChannel.LEDChannelA); // Green
        mCanifier.setLEDOutput(rgb.getBPercent(), CANifier.LEDChannel.LEDChannelC); // Blue
    }


    public void turnOffLED()
    {
        setLED(LEDColor.NONE);
    }


    @Override
    public void readInputs(double pNow) {}

    @Override
    public void setOutputs(double pNow) {

    }

    public void shutdown(double pNow) {
        // TODO Auto-generated method stub
        this.turnOffLED();
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }


}