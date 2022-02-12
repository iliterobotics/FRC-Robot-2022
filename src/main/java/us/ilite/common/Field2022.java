package us.ilite.common;

import us.ilite.robot.hardware.LidarLite;

public class Field2022 {
//    /**
//     * Determines whether the input azimuth falls within the boundary conditions given by the regression trendline
//     * a = 0.00006 * x^2 - 0.0241*x + 18.906
//     *
//     * This was determined emperically via OnShape CAD and has a R^2 value of 0.978 with a safety margin of 0.25 degrees
//     *
//     * Note that the Azimuth assumes the angle to the INNER GOAL! This is not a direct reading from the LimeLight or other
//     * vision system. See us.ilite.common.lib.util.Utils::calculateAngleOffset(...). Also note that it uses distance
//     * from the OUTER GOAL, which is also emperically determined via OnShape.
//     *
//     * @param pAzimuth - Azimuth from the robot to the INNER goal
//     * @param pDistance - Distance from the robot to the OUTER goal
//     * @return TRUE if the magnitude of the azimuth is less than or equal to the regression line for the given distance
//     */
//    public static boolean canHitInnerGoal(Angle pAzimuth, Distance pDistance) {
//        return abs(pAzimuth.degrees()) <= 0.00006 * pow(pDistance.inches(),2) - 0.0241*pDistance.inches() + 18.906;
//    }

    public static double distanceFromGoal(LidarLite pLidar) {
        double lidarDistance = pLidar.getDistance();
        double lidarAngle = Math.PI/6;
        double lidarHeight = 24;
        double deltaHeight = FieldElement.HUB_UPPER.height() - lidarHeight;
        double theta2 = Math.acos(deltaHeight/lidarDistance) + Math.PI/2;
        double hubEdgeToCenter = FieldElement.HUB_LOWER.width()/2;
        double distanceToOuter = pLidar.getDistance()*Math.cos(lidarAngle);
        double slantDistanceToCenter = Math.sqrt(hubEdgeToCenter*hubEdgeToCenter + distanceToOuter*distanceToOuter -2*distanceToOuter*hubEdgeToCenter*Math.cos(theta2));
        return slantDistanceToCenter;
    }

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
        LINE                    (0d,0d), //TODO fix the pipelines and distances
        HUB_UPPER               (104,48),
        HUB_LOWER               (67,24),
        TARGET_ZOOM             (0d,0d),
        BALL                    (0d,0d);

        private final double height;
        private final int pipeline;
        private final double width;

        // TODO confirm if we are using a limelight or lidar

        // Limelight-based FieldElement
        private FieldElement(int pPipeline, double pHeight, double pWidth){
            width = pWidth;
            height = pHeight;
            pipeline = pPipeline;
        }

        // Lidar-based FieldElement
        private FieldElement(double pHeight, double pWidth) {
            width = pWidth;
            height = pHeight;
            pipeline = 0;
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
