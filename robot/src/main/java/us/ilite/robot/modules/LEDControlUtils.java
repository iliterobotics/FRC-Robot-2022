package us.ilite.robot.modules;
import us.ilite.robot.modules.DJSpinnerModule.*;

public class LEDControlUtils {

    public static class RGB {
        public int mR;
        public int mG;
        public int mB;

        public RGB(int pR, int pG, int pB) {
            // Value range for each color is 0-255, we'll enforce this with a module divide
            this.mR = pR % 256;
            this.mG = pG % 256;
            this.mB = pB % 256;
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
        DEFAULT( 0, 0, 0 );

        private RGB rgb;

        LEDColor( int pR, int pG, int pB ) {
            this.rgb = new RGB(pR, pG, pB);
        }

        public RGB getColor() {
            return this.rgb;
        }
    }

    /**
     * Message to communicate to the LEDControl module
     */
    public enum Message {
        ON_BLUE( LEDColor.BLUE, false ),
        ON_RED( LEDColor.RED, false ),
        ON_GREEN( LEDColor.GREEN, false ),
        ON_YELLOW( LEDColor.YELLOW, false ),
        FINISHED_ON_BLUE( LEDColor.BLUE, true ),
        FINISHED_ON_RED( LEDColor.RED, true ),
        FINISHED_ON_GREEN( LEDColor.GREEN, true ),
        FINISHED_ON_YELLOW( LEDColor.YELLOW, true ),
        NONE(LEDColor.DEFAULT, false),

        CURRENT_LIMITING( LEDColor.RED, false),
        VISION_TRACKING( LEDColor.GREEN, false);

        final LEDColor color;
        // pulse speed in milliseconds, 0 = on solid
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
}
