package us.ilite.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Angle;
import us.ilite.common.config.Settings;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.modules.FalconDriveModule;
import us.ilite.common.types.sensor.EGyro;
import edu.wpi.first.math.controller.PIDController;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import us.ilite.robot.modules.NeoDriveModule;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.drive.EDriveData.DESIRED_TURN_PCT;

public class TurnToDegree implements ICommand {

  private ILog mLogger = Logger.createLog( this.getClass() );

  private static final double kTIMEOUT = 9999.9;
  private static final double kMIN_POWER = 0.0;
  private static final double kMAX_POWER = 0.5;
  private static final int kMIN_ALIGNED_COUNT = 25;
  private final double mAllowableError;

  private double mOutput = 0.0;
  private double mStartTime;
  private int mAlignedCount;

  private Rotation2d mInitialYaw, mTurnAngle, mTargetYaw;
  private PIDController pid;

  public TurnToDegree(Rotation2d pTurnAngle, double pAllowableError) {
    this.mTurnAngle = pTurnAngle;
    this.mAllowableError = pAllowableError;
  }

  @Override
  public void init( double pNow ) {
    mStartTime = pNow;

    mInitialYaw = getYaw();
    double init  = Robot.DATA.drivetrain.get( ACTUAL_HEADING_DEGREES );
    double targetAngle = init + mTurnAngle.getDegrees();
    mTargetYaw = mInitialYaw.rotateBy( mTurnAngle );

    SmartDashboard.putNumber("Target Gyro Heading", targetAngle);

    // PIDController configuration
    pid = NeoDriveModule.kTurnToProfileGains.generateWPIPIDController();
    pid.setSetpoint( targetAngle);

    mAlignedCount = 0;
  }

  public boolean update( double pNow ) {
    mOutput = pid.calculate(  Robot.DATA.drivetrain.get( ACTUAL_HEADING_DEGREES ) );
//    mOutput += Math.signum( mOutput ) * NeoDriveModule.kTurnToProfileGains.F;

    // Keep track of time on target
//    if ( ( Math.abs( pid.getPositionError() ) <= Math.abs( mAllowableError ) ) ) {
//      mAlignedCount++;
//    } else {
//      mAlignedCount = 0;
//    }

    // End if on target for 25 counts
//    if ( mAlignedCount >= kMIN_ALIGNED_COUNT || pNow - mStartTime > kTIMEOUT ) {
//      Robot.DATA.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
//      Robot.DATA.drivetrain.set(EDriveData.NEUTRAL_MODE, ECommonNeutralMode.BRAKE);
//      Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0d);
//      Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, 0d);
//      mLogger.info( "Turn finished" );
//      return true;
//    }

    // Apply output, log, and return false for unfinished
    Robot.DATA.drivetrain.set(NEUTRAL_MODE, ECommonNeutralMode.BRAKE);
    Robot.DATA.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
   // Robot.DATA.drivetrain.set(DESIRED_THROTTLE_PCT, 0.05);
    Robot.DATA.drivetrain.set(DESIRED_TURN_PCT, mOutput);
    SmartDashboard.putNumber("Turn Power", mOutput);
    SmartDashboard.putNumber("Turn To Degree Error", pid.getPositionError());
    mLogger.info( "Target: " + mTargetYaw + " Yaw: " + getYaw() + "\n" );
    return false;
  }

  private Rotation2d getYaw() {
    // TODO - was this inverted?
    return Rotation2d.fromDegrees( Robot.DATA.drivetrain.get( ACTUAL_HEADING_DEGREES ) );
  }

  @Override
  public void shutdown( double pNow ) {

  }

}