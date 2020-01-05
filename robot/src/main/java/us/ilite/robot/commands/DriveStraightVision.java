package us.ilite.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.robot.hardware.IMU;
import us.ilite.robot.modules.Drive;

public class DriveStraightVision extends CommandQueue {

    private IMU mInitialImu;

    public DriveStraightVision(Drive pDrive, IMU pImu, Data pData, DriveStraight.EDriveControlMode pDriveControlMode, double pDistanceToDrive) {
        mInitialImu = pDrive.getDriveHardware().getImu();
        setCommands(
                new FunctionalCommand(() -> pDrive.getDriveHardware().setImu(pImu)),
                new DriveStraight(pDrive, pData, pDriveControlMode, pDistanceToDrive)
                        .setTargetHeading(Rotation2d.fromDegrees(0.0))
                        .setHeadingGains(Settings.kTargetAngleLockGains),
                new FunctionalCommand(() -> pDrive.getDriveHardware().setImu(mInitialImu))
        );
    }

}
