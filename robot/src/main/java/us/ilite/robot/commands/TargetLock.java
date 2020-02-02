package us.ilite.robot.commands;

import com.flybotix.hfr.codex.Codex;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.IFieldComponent;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.DriveModule;
import us.ilite.robot.modules.DriveMessage;
import us.ilite.robot.modules.EDriveState;
import us.ilite.robot.modules.IThrottleProvider;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

public class TargetLock implements ICommand {

    private static final double kTURN_POWER = 0.2;
    private static final int kAlignCount = 10;
    private static final double kTargetAreaScalar = 1.0;

    private DriveModule mDrive;
    private ITargetDataProvider mCamera;
    // Different throttle providers give us some control over behavior in autonomous
    private IThrottleProvider mTargetSearchThrottleProvider, mTargetLockThrottleProvider;
    private IFieldComponent mTrackingType;

    private double mAllowableError, mPreviousTime, mOutput = 0.0;

    private boolean mEndOnAlignment = true;
    private int mAlignedCount = 0;
    private boolean mHasAcquiredTarget = false;
    private boolean mStopWhenTargetLost = true;

    public TargetLock(double pAllowableError, IFieldComponent pTrackingType, ITargetDataProvider pCamera, IThrottleProvider pThrottleProvider) {
        this(pAllowableError, pTrackingType, pCamera, pThrottleProvider, true);
    }

    public TargetLock(double pAllowableError, IFieldComponent pTrackingType, ITargetDataProvider pCamera, IThrottleProvider pThrottleProvider, boolean pEndOnAlignment) {
        this.mAllowableError = pAllowableError;
        this.mCamera = pCamera;
        this.mTargetSearchThrottleProvider = pThrottleProvider;
        this.mTargetLockThrottleProvider = pThrottleProvider;
        this.mEndOnAlignment = pEndOnAlignment;
    }

    @Override
    public void init(double pNow) {
        System.out.println("++++++++++++++++++++++++++TARGET LOCKING++++++++++++++++++++++++++++++++++++\n\n\n\n");

        mHasAcquiredTarget = false;
        mAlignedCount = 0;

        Robot.DATA.drivetrain.set(EDriveData.DESIRED_STATE, EDriveState.TARGET_ANGLE_LOCK);
        Robot.DATA.drivetrain.set(EDriveData.TARGET_TRACKING_THROTTLE, 0.0);

        this.mPreviousTime = pNow;
    }

    @Override
    public boolean update(double pNow) {
        RobotCodex<ELimelightData> currentData = mCamera.getTargetingData();

        Robot.DATA.drivetrain.set(EDriveData.TARGET_TRACKING_THROTTLE, mTargetLockThrottleProvider.getThrottle() * Settings.Input.kSnailModePercentThrottleReduction);

        if(currentData != null && currentData.isSet(ELimelightData.TV) && currentData.isSet(ELimelightData.TX)) {
            mHasAcquiredTarget = true;

            mAlignedCount++;
            if(mEndOnAlignment && Math.abs(currentData.get(ELimelightData.TX)) < mAllowableError && mAlignedCount > kAlignCount) {
                System.out.println("FINISHED");
                // Zero drivetrain outputs in shutdown()
                return true;
            }

        // If we've already seen the target and lose tracking, exit.
        } else if(mHasAcquiredTarget && !currentData.isSet(ELimelightData.TV)) {
            return true;
        }
//        if(!mHasAcquiredTarget){
//            System.out.println("OPEN LOOP");
//            mAlignedCount = 0;
//            //if there is no target in the limelight's pov, continue turning in direction specified by SearchDirection
//            mDrive.setDriveMessage(
//                new DriveMessage(
//                    mTargetSearchThrottleProvider.getThrottle() + (mTrackingType.getTurnScalar() * kTURN_POWER),
//                    mTargetSearchThrottleProvider.getThrottle() + (mTrackingType.getTurnScalar() * -kTURN_POWER),
//                    ECommonControlMode.PERCENT_OUTPUT
//                ).setNeutralMode(ECommonNeutralMode.BRAKE)
//            );
//        }

        mPreviousTime = pNow;
        
         //command has not completed
        return false;                                                      
    }

    @Override
    public void shutdown(double pNow) {
        Robot.DATA.drivetrain.set(EDriveData.DESIRED_STATE, EDriveState.NORMAL);
        mDrive.setDriveMessage(DriveMessage.kNeutral);
    }

    public TargetLock setTargetLockThrottleProvider(IThrottleProvider pThrottleProvider) {
        this.mTargetLockThrottleProvider = pThrottleProvider;
        return this;
    }

    public TargetLock setTargetSearchThrottleProvider(IThrottleProvider pThrottleProvider) {
        this.mTargetSearchThrottleProvider = pThrottleProvider;
        return this;
    }
    public TargetLock setStopWhenTargetLost(boolean pStopWhenTargetLost) {
        this.mStopWhenTargetLost = pStopWhenTargetLost;
        return this;
    }

}