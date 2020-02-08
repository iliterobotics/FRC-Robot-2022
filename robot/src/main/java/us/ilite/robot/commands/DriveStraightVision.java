package us.ilite.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import us.ilite.common.Angle;
import us.ilite.common.Data;
import us.ilite.common.Distance;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.IMU;
import us.ilite.robot.modules.DriveModule;

import static us.ilite.common.types.sensor.EGyro.HEADING_DEGREES;

public class DriveStraightVision extends DriveStraight {


    public DriveStraightVision(DriveStraight.EDriveControlMode pDriveControlMode, Distance pDistanceToDrive) {
        super(pDriveControlMode, pDistanceToDrive);
    }

    @Override
    protected Angle getHeading() {
        return Angle.fromDegrees(Robot.DATA.limelight.get(ELimelightData.CALC_ANGLE_TO_TARGET));
    }

}
