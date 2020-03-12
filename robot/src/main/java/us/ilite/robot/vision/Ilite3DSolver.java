package us.ilite.robot.vision;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.lib.util.Utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import static java.lang.Math.*;

/**
 * Creates a 3D vision processor for a specific goal. Handles all projections internally.
 * Credit for pieces of this class goes to FRC 254, with tips from Jared on what's actually useful in certain situations
 *
 * ILITE COORDINATE SYSTEM IS GOAL-CENTRIC. This is so we can match the CAD sketches of field geometry.
 * Much of this math comes from https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles, but with changes
 * to the axes as follows:
 * x runs the field left/right. Positive X = Right from the perspective of the drivers
 * y runs the length of the field, forward/reverse. Positive Y = away from the drivers; Negative Y = towards the drivers
 * z is HEIGHT, normal to the playing surface. Z is always positive (supposing our laws of physics are correct)
 */
public class Ilite3DSolver {
    private final Distance mNormalOffset;
    private final Distance mGoalHeight;
    private final Distance mGoalWidth;
    private final CameraConfig mConfig;

    public static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::getX);
    public static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::getY);

    private Distance mRangeToGoalCenter = null;
    private Angle mAbsoluteAzimuthToGoalCenterThetaT = null;
    private Distance mAbsoluteX = null;
    private Distance mAbsoluteY = null;
    private Angle mOffsetAngle = null;
    private Distance mLeftRange = null;
    private Distance mRightRange = null;

    /**
     * Creates a Ilite3D Solver with the input parameters as the target objectives
     * @param pCameraConfig - Camera configuration that has FOV, resolution, etc
     * @param pGoalHeight - Height of the physical target (and the resulting corner points), from the floor
     * @param pGoalWidth - Width of the physical target (corresponding to corner points)
     */
    public Ilite3DSolver(CameraConfig pCameraConfig, Distance pGoalHeight, Distance pGoalWidth) {
        this(pCameraConfig, pGoalHeight, pGoalWidth, null);
    }

    /**
     * Creates a Ilite3D Solver with the input parameters as the target objectives
     * @param pCameraConfig - Camera configuration that has FOV, resolution, etc
     * @param pGoalHeight - Height of the physical target (and the resulting corner points), from the floor
     * @param pGoalWidth - Width of the physical target (corresponding to corner points)
     * @param pNormalOffset - The 3D normal offset which may need to be adjusted for. Set to 0 if you just want to aim at the goal.
     */
    public Ilite3DSolver(CameraConfig pCameraConfig, Distance pGoalHeight, Distance pGoalWidth, Distance pNormalOffset) {
        mConfig = pCameraConfig;
        mGoalHeight = pGoalHeight;
        mGoalWidth = pGoalWidth;
        mNormalOffset = pNormalOffset;
    }


    /**
     * @return the absolute x position of the robot relative to a fixed goal. For a game like 2020, x() represents absolute
     * field localization
     */
    public Distance x() { return mAbsoluteX; }

    /**
     * @return the absolute y position of the robot relative to a fixed goal. For a game like 2020, y() represents absolute
     * field localization
     */
    public Distance y() { return mAbsoluteY; }

    /**
     *
     * @return the absolute distance from the camera lens to the center of the target goal
     */
    public Distance range() { return mRangeToGoalCenter; }

    /**
     * @return the goal-oriented absolute azimuth from the robot to the goal. Think of this like a goal-relative gyro.
     */
    public Angle azimuth() { return mAbsoluteAzimuthToGoalCenterThetaT; }

    /**
     * Returns the angle adjustment from the azimuth that is needed to hit the normal offset distance
     * @return
     */
    public Angle offsetAzimuth() { return mOffsetAngle; }

    public Distance leftRange() {return mLeftRange;}

    public Distance rightRange() {return mRightRange;}

    /**
     * Updates this objects absolute pose based upon the corners of a target. It is assumed that the center of the goal
     * is origin (0,0), with orientation such that a perfectly-aligned robot placing an element into the goal is
     * perpendicular to the goal. Updates absolute x, y, distance, and azimuth (symmetric) relative to the origin.
     * Also calculates the angle offset, if a normal offset value is present.
     *
     * @param pCorners these corners both represent points on a line that is perfectly horizontal on the field, even if
     *                 the pixel points themselves do not form a horizontal line in the camera image. NOTE - the list will
     *                 be sorted so that the left-most point is first in the list and the right-most point is the last in the list.
     */
    public void updatePoseToGoal(List<Translation2d> pCorners) {
        if(pCorners == null || pCorners.size() < 2) {
            return;
        }
        List<Translation2d> copy = new ArrayList<>(pCorners);
        // Sort left/right so the first element is the left-most point and the last element is the right-most point
        copy.sort(xSort);
        mLeftRange = getRangeToPixelPoint(copy.get(0));
        mRightRange = getRangeToPixelPoint(copy.get(copy.size()-1));

        // Third leg of the triangle is the goal width, which should correspond to the left-most and right-most points
        // https://en.wikipedia.org/wiki/True_range_multilateration
        // Let origin (0,0) = the center of the goal and U = goalwidth
        // Then coordinates C1 = right-most point of the goal = (goalwidth/2, 0) and C2 = (-goalwidth/2, 0)
        // Let W = goalwidth/2 --> https://www.wolframalpha.com/input/?i=solve%28L%5E2+-+%28-w-x%29%5E2+%3D+R%5E2+-+%28w-x%29%5E2%2C+x%29
        mAbsoluteX = Distance.fromInches((pow(mLeftRange.inches(),2d) - pow(mRightRange.inches(),2d)) / (2 * mGoalWidth.inches())); // If negative, then we are left of the goal
        mAbsoluteY = Distance.fromInches(sqrt(pow(mLeftRange.inches(),2d) - pow(-mGoalWidth.inches() /2 - mAbsoluteX.inches(), 2)));
        mRangeToGoalCenter = Distance.hypot(mAbsoluteX,mAbsoluteY);
        mAbsoluteAzimuthToGoalCenterThetaT = Distance.atan2(mAbsoluteY, mAbsoluteX);
        if(mNormalOffset != null) {
            mOffsetAngle = Utils.calculateAngleOffsetY(
                    mAbsoluteAzimuthToGoalCenterThetaT,
                    mRangeToGoalCenter,
                    mNormalOffset
            );
        }
    }

    /**
     * Returns a relative range from the camera lens to a target identified as a point in the camera's image. This method
     * accounts for the spherical skewe that comes with high elevation angles and a point that is not at the center of the
     * camera lens.
     * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/6
     * @param pCorner a point in units of pixels the given camera configuration
     * @return a distance, in inches, to the pixel value
     */
    public Distance getRangeToPixelPoint(Translation2d pCorner) {
        // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
        double nx = ((getX(pCorner) - mConfig.hResolution/2.0) / (mConfig.hResolution/2.0));
        double nz = ((getY(pCorner) - mConfig.vResolution/2.0) / (mConfig.vResolution/2.0));
        double x = mConfig.vpw/2.0 * nx; // Left/right
        double z = mConfig.vph/2.0 * nz; // Height
        double result = Double.NaN;
        // 254-2019 RobotState::getCameraToVisionTargetPose()
        Rotation2d elevation = Rotation2d.fromDegrees(mConfig.elevation_deg());
        Translation2d xy_translation = new Translation2d(1.0, z).rotateBy(elevation);
        // Different from 254's - have to convert this to radians
//        x = toRadians(x);
        double y = xy_translation.getX();
        z = xy_translation.getY();
//        System.out.println(String.format("x=%f\ty=%f\tz=%f",x,y,z));

        // find intersection with the goal
        double differential_height = mConfig.lensheight_in() - mGoalHeight.inches();
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = hypot(x, y) * scaling;
            // The rest of this seems redundant, but it isn't - this accounts for spherical effect of camera lenses
            // described in the CD post above
            Rotation2d angle = new Rotation2d(y, x);
            result = hypot(distance * cos(angle.getRadians()), distance * sin(angle.getRadians()));
        }

        return Distance.fromInches(result);
    }

    /**
     * Utility method to handle camera inversion
     * @param pCorner - the detected point
     * @return the y pixel value, accounting for inversion
     */
    private double getY(Translation2d pCorner) {
        if(mConfig.isInverted()) {
            return mConfig.vResolution - pCorner.getY();
        } else {
            return pCorner.getY();
        }
    }

    /**
     * Utility method to handle camera inversion
     * @param pCorner - the detected point
     * @return the x pixel value, accounting for inversion
     */
    private double getX(Translation2d pCorner) {
        if(mConfig.isInverted()) {
            return mConfig.hResolution - pCorner.getX();
        } else {
            return pCorner.getX();
        }
    }
}
