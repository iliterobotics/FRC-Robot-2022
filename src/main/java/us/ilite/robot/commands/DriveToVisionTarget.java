package us.ilite.robot.commands;

import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.util.Utils;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.modules.DriveModule;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

/**
 * Untested. Should turn more aggressively when further away from the target.
 */
@Deprecated
public class DriveToVisionTarget implements ICommand {

    private static final double kMaxTargetAngle = 27.0;
    private static final double kMaxTargetArea = -1.0;
    private static final double kMinTargetArea = -1.0;

    // Stop adjusting the target angle when we are nearer than this amount to the target
    private static final double kAngleAdjustDistanceThreshold = 0.0;
    private static final double kAngleToTarget_kP = 0.0;

    private final PIDController mHeadingController = new PIDController(Settings.kTargetAngleLockGains, -kMaxTargetAngle, kMaxTargetAngle, Settings.kControlLoopPeriod);
    private final PIDController mDistanceController = new PIDController(Settings.kTargetDistanceLockGains, kMinTargetArea, kMaxTargetAngle, Settings.kControlLoopPeriod);

    private ITargetDataProvider mTargetDataProvider;
    private DriveModule mDrive;
    private Data mData;

    private double mInitialTargetAngle;
    private boolean mHasAcquiredTarget = false;

    public DriveToVisionTarget(ITargetDataProvider pTargetDataProvider, DriveModule mDrive, Data mData) {
        this.mTargetDataProvider = pTargetDataProvider;
        this.mDrive = mDrive;
        this.mData = mData;
    }

    @Override
    public void init(double pNow) {
        mHasAcquiredTarget = false;
        mHeadingController.reset();
        mInitialTargetAngle = mData.limelight.get(ELimelightData.TX);
    }

    @Override
    public boolean update(double pNow) {

        /*
         We either drove into the target and lost sight of it (meaning we're done) or we never had sight of it to begin with (in which case we quit)
         */
        if(!mData.limelight.isSet(ELimelightData.TV)) {
            return true;
        }

        // Target "distance" is just area - min_area, clamped to a maximum and minimum value
        double distanceFromTarget = Utils.clamp(0.0 /*mData.limelight.get(EVisionGoal2020.TA)*/ - kMinTargetArea, kMinTargetArea, kMaxTargetArea);
        double angleToTarget = mData.limelight.get(ELimelightData.TX);

        // Only adjust target angle if we are far away
        if(distanceFromTarget < kAngleAdjustDistanceThreshold) {
            /*
            Turn more aggressively when we are further away from the target.
            If the angle to target is negative (to the left), adjust the target angle to the right.
            If the angle to target is positive (to the right), adjust the target angle to the left.
             */
            double targetAngle = 0.0 - (Math.signum(angleToTarget) * kAngleToTarget_kP * distanceFromTarget);
            mHeadingController.setSetpoint(targetAngle);
        } else {
            mHeadingController.setSetpoint(0.0);
        }

        mDistanceController.setSetpoint(0.0);

        double turn = mHeadingController.calculate(angleToTarget, pNow);
        double throttle = mDistanceController.calculate(distanceFromTarget, pNow);

//        mDrive.setDriveMessage(new DriveMessage().throttle(throttle).turn(turn).normalize());

        return false;
    }

    @Override
    public void shutdown(double pNow) {

    }

}
