package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;

public class FiveBallController extends ThreeBallController {
    private DriveStraight mFourthLeg = new DriveStraight(Distance.fromFeet(12d));
    private boolean mFourthLegComplete = false;
    private DriveStraight mFifthLeg = new DriveStraight(Distance.fromFeet(-12d));
    private boolean mFifthLegComplete = false;
    public void initialize(Trajectory pTrajectory) {
        super.initialize(null);
    }

    private static double
            kFirstFireTime = kThirdLegTimeEnd + 1.0,
            kFourthLegTimeEnd = kFirstFireTime + 3.0,
            kHumanPlayerLoadTime = kFourthLegTimeEnd + 3.0,
            kFifthLegTime = kHumanPlayerLoadTime + 3.0,
            kSecondFireTime = kFifthLegTime + 1.0;

    public void updateImpl() {
        super.updateImpl();
    }
}
