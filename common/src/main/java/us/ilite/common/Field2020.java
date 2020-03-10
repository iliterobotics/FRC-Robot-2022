package us.ilite.common;

import static java.lang.Math.*;


public class Field2020 {

    /**
     * Determines whether the input azimuth falls within the boundary conditions given by the regression trendline
     * a = 0.00006 * x^2 - 0.0241*x + 18.906
     *
     * This was determined emperically via OnShape CAD and has a R^2 value of 0.978 with a safety margin of 0.25 degrees
     *
     * Note that the Azimuth assumes the angle to the INNER GOAL! This is not a direct reading from the LimeLight or other
     * vision system. See us.ilite.common.lib.util.Utils::calculateAngleOffset(...). Also note that it uses distance
     * from the OUTER GOAL, which is also emperically determined via OnShape.
     *
     * @param pAzimuth - Azimuth from the robot to the INNER goal
     * @param pDistance - Distance from the robot to the OUTER goal
     * @return TRUE if the magnitude of the azimuth is less than or equal to the regression line for the given distance
     */
    public static boolean canHitInnerGoal(Angle pAzimuth, Distance pDistance) {
        return abs(pAzimuth.degrees()) <= 0.00006 * pow(pDistance.inches(),2) - 0.0241*pDistance.inches() + 18.906;
    }

    public enum Distances{
        INITIATION_LINE_TO_COLOR_WHEEL(0);

        public double mDistance;

        Distances(double pDist) {
            mDistance = pDist;
        }
    }

    /**
     * Any trackable field component for 2020's game.
     */
    public enum FieldElement implements IFieldComponent {

//        TARGET(12d,0),
        LINE                    (5,0d,      0d), //TODO detemine why these they have the same pipeline
        OUTER_GOAL_UPPER_CORNERS(1,98.25,   36.641),
        OUTER_GOAL_LOWER_CORNERS(1,83.25,   17.321),
        POWER_CELL              (1,0.0,     0d),
        TARGET_ZOOM             (3,0d,      0d),
        BALL                    (3,0d,      0d),
        BALL_DUAL               (5,0d,      0d),
        BALL_TRI                (6,0d,      0d);

        private final double height;
        private final int pipeline;
        private final double width;

        private FieldElement(int pPipeline, double pHeight, double pWidth){
            width = pWidth;
            height = pHeight;
            pipeline = pPipeline;
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
