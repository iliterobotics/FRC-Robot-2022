package us.ilite.robot.commands;

import com.flybotix.hfr.codex.RobotCodex;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.IThrottleProvider;

import static us.ilite.common.types.ELimelightData.*;

public class TargetLock implements ICommand {

    private static final double kTURN_POWER = 0.2;
    private static final int kAlignCount = 10;
    private static final double kTargetAreaScalar = 1.0;

//    private DriveModule mDrive;
//    private ITargetDataProvider mCamera;
//    // Different throttle providers give us some control over behavior in autonomous
//    private IThrottleProvider mTargetSearchThrottleProvider, mTargetLockThrottleProvider;
//    private IFieldComponent mTrackingType;

    private double mAllowableError, mPreviousTime, mOutput = 0.0;

    private boolean mEndOnAlignment = true;
    private int mAlignedCount = 0;
    private boolean mHasAcquiredTarget = false;
    private boolean mStopWhenTargetLost = true;

//    public TargetLock(double pAllowableError, IFieldComponent pTrackingType, ITargetDataProvider pCamera, IThrottleProvider pThrottleProvider) {
//        this(pAllowableError, pTrackingType, pCamera, pThrottleProvider, true);
//    }

    public TargetLock(/*double pAllowableError, IFieldComponent pTrackingType, ITargetDataProvider pCamera, IThrottleProvider pThrottleProvider, boolean pEndOnAlignment*/) {
//        this.mAllowableError = pAllowableError;
//        this.mCamera = pCamera;
//        this.mTargetSearchThrottleProvider = pThrottleProvider;
//        this.mTargetLockThrottleProvider = pThrottleProvider;
//        this.mEndOnAlignment = pEndOnAlignment;
    }

    @Override
    public void init(double pNow) {
        System.out.println("++++++++++++++++++++++++++TARGET LOCKING++++++++++++++++++++++++++++++++++++\n\n\n\n");

        mHasAcquiredTarget = false;
        mAlignedCount = 0;

//        mDrive.setTargetAngleLock();
//        mDrive.setTargetTrackingThrottle(0);

//        Robot.DATA.drivetrain.set(EDriveData.);   //TODO wait for drivetrain state code

        this.mPreviousTime = pNow;
    }

    @Override
    public boolean update(double pNow) {
        RobotCodex<ELimelightData> currentData = Robot.DATA.limelight;

//        mDrive.setTargetTrackingThrottle(mTargetLockThrottleProvider.getThrottle() * Settings.Input.kSnailModePercentThrottleReduction);

        if(currentData.isSet(TV) && currentData.isSet(TX)) {
            mHasAcquiredTarget = true;

            mAlignedCount++;
            if(mEndOnAlignment && Math.abs(currentData.get(TX)) < mAllowableError && mAlignedCount > kAlignCount) {
                System.out.println("FINISHED");
                // Zero drivetrain outputs in shutdown()
                return true;
            }

        // If we've already seen the target and lose tracking, exit.
        } else if(mHasAcquiredTarget && !currentData.isSet(TV)) {
            return true;
        }

        mPreviousTime = pNow;
        
         //command has not completed
        return false;                                                      
    }

    @Override
    public void shutdown(double pNow) {
//        Robot.DATA.drivetrain.set(EDriveData.); //TODO wait for drivetrain state code
//        mDrive.setNormal();
//        mDrive.setDriveMessage(DriveMessage.kNeutral);
    }

    public TargetLock setTargetLockThrottleProvider(IThrottleProvider pThrottleProvider) {
//        this.mTargetLockThrottleProvider = pThrottleProvider;
        return this;
    }

    public TargetLock setTargetSearchThrottleProvider(IThrottleProvider pThrottleProvider) {
//        this.mTargetSearchThrottleProvider = pThrottleProvider;
        return this;
    }
    public TargetLock setStopWhenTargetLost(boolean pStopWhenTargetLost) {
        this.mStopWhenTargetLost = pStopWhenTargetLost;
        return this;
    }

}