/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package us.ilite.robot.modules.targetData;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.IFieldComponent;

/**
 * Add your docs here.
 */
public interface ITargetDataProvider {
    RobotCodex<ELimelightData> getTargetingData();

    double getCameraHeightIn();

    double getCameraAngleDeg();

    double getCameraToBumperIn();

    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) parameter from the limelight
     */
    double ty();

    /**
     * Calculate the distance to the currently tracked target.
     * @return Distance to target
     */
    default double calcTargetDistance(double pTargetHeight) {
        // d = h/(tan(Ac - ty)) - db
        // hc = measured height of camera lens: Settings.llCameraHeightIn
        // ht = height of the target being tracked: targetHeight
        // h = hc - ht = Height of triangle for distance calculation
        // d = distance from robot bumper to center of target bottom, this is what we're calculating
        // db = measured distance from camera lens to robot bumper: Settings.llCameraToBumperIn
        // Ac = camera angle needed for calculating the distance: Settings.llCameraAngleDeg
        // ty = Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) parameter from the limelight

        // we read the lime light values from mCurrentTarget, but this may be null if update is not
        // called for the first time

        double d = (getCameraHeightIn() - pTargetHeight) /
                Math.tan( getCameraAngleDeg() - ty() ) -
                getCameraToBumperIn();

        return d;
    }

    /**
     * Calculate the distance to the currently tracked target by target type
     * @param target
     * @return Distance to target
     */
    public default double calcTargetDistance( IFieldComponent target ) {
        return this.calcTargetDistance( target.height() );
    }

}
