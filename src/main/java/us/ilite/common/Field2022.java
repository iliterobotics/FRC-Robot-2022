package us.ilite.common;

import us.ilite.robot.hardware.LidarLite;

public class Field2022 {

    public enum Distances{
        TARMAC_TO_HUB(84.25),
        HUB_RING_DIAMETER(306),
        BALL_TO_HUB(126.5),
        TARGETTING_OFFSET(21.0);

        public Distance mDistance;

        Distances(double pDist) {
            mDistance = Distance.fromInches(pDist);
        }
    }

    /**
     * Any trackable field component for 2020's game.
     */
    public enum FieldElement implements IFieldComponent {
        HUB_UPPER               (104,48, 0),
//        TARGET_ZOOM             (0d,0d),
        NONE                    (0,0, 1);

        // This is done in inches -- straight from game manual
        private final double height;
        private final double width;
        private int pipeline;

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth, int pPipeline){
            width = pWidth;
            height = pHeight;
            pipeline = pPipeline;
        }

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth){
            width = pWidth;
            height = pHeight;
            pipeline = 1;
        }

        @Override
        public double width() { return width; }

        @Override
        public double height() {
            return height;
        }

        @Override
        public int pipeline() {
            return pipeline;
        }

        public int id() { return ordinal(); }
    }
}
