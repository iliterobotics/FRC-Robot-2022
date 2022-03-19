package us.ilite.robot.commands;

import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.Robot;

@Deprecated
public class DriveStraightVision extends DriveStraight {


    public DriveStraightVision(DriveStraight.EDriveControlMode pDriveControlMode, Distance pDistanceToDrive) {
        super(pDistanceToDrive);
    }

    @Override
    protected Angle getHeading() {
        return Angle.fromDegrees(Robot.DATA.limelight.get(ELimelightData.TARGET_AZIMUTH_deg));
    }

}
