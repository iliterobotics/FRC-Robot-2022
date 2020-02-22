package us.ilite.robot.commands;

import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.Robot;

public class DriveStraightVision extends DriveStraight {


    public DriveStraightVision(DriveStraight.EDriveControlMode pDriveControlMode, Distance pDistanceToDrive) {
        super(pDriveControlMode, pDistanceToDrive);
    }

    @Override
    protected Angle getHeading() {
        return Angle.fromDegrees(Robot.DATA.goaltracking.get(ELimelightData.CALC_ANGLE_TO_TARGET));
    }

}
