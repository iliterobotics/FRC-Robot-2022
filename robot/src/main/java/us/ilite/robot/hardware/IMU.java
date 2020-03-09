package us.ilite.robot.hardware;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

import org.apache.commons.lang3.ArrayUtils;

import us.ilite.common.Angle;
import us.ilite.common.lib.util.FilteredAverage;

public abstract  class IMU {

  protected static final Rotation2d ZERO = new Rotation2d(0d);
  public enum Axis {
    YAW,
    PITCH,
    ROLL
  }

  protected static final double[] DEFAULT_GAINS = new double[]{1.0};

  //Collision Threshold => Temporary Value
  protected transient double mCollisionThreshold_DeltaG;
  protected transient final FilteredAverage mAccelerationX;
  protected transient final FilteredAverage mAccelerationY;
  protected transient double mJerkX = 0d;
  protected transient double mJerkY = 0d;
  protected double mLastUpdate = 0d;
  protected double dt = 0d;
  protected final Clock mClock;
  protected Rotation2d mLastYaw = Rotation2d.fromDegrees(0d);

  public IMU(Clock pRobotClock, List<Double>pFilterGains) {
    this(pRobotClock, ArrayUtils.toPrimitive(pFilterGains.toArray(new Double[0])));
  }
  
  public IMU(Clock pRobotClock, double[] pFilterGains) {
    mClock = pRobotClock;
    mAccelerationX = new FilteredAverage(pFilterGains);
    mAccelerationY = new FilteredAverage(pFilterGains);
  }

  /**
   * Sets the g-force threshold.  This is a tuneable parameter between different robots & years.
   * @param pCollisionThreshold_DeltaG - new g-force parameter
   */
  public final void setCollisionThreshold_DeltaG(double pCollisionThreshold_DeltaG) {
    mCollisionThreshold_DeltaG = pCollisionThreshold_DeltaG;
  }
  
  public Rotation2d get(Axis pAxis) {
    switch(pAxis) {
    case PITCH:
      return getPitch();
    case ROLL:
      return getRoll();
    case YAW:
      return getYaw();
    default:
      return getYaw();
    }
  }
  
  /**
   * Pre-populates the filters & calculated values so it's done only once per cycle
   */
  public void update() {
    mLastYaw = getYaw();
    updateSensorCache();
    double currentAccelX = getRawAccelX();
    double currentAccelY = getRawAccelY();
    
    mJerkX = (currentAccelX - mAccelerationX.getAverage()) / (mClock.dt());
    mJerkY = (currentAccelY - mAccelerationY.getAverage()) / (mClock.dt());
    
    mAccelerationX.addNumber(currentAccelX);
    mAccelerationY.addNumber(currentAccelY);
  }
  
  public abstract Rotation2d getYaw();
  public abstract Rotation2d getPitch();
  public abstract Rotation2d getRoll();
  public abstract void zeroAll();
  protected abstract double getRawAccelX();
  protected abstract double getRawAccelY();

  /**
   * Update the local cache of sensor values. Store these locally and
   * use that cache for every get() required by this IMU class. It is NOT
   * recommended to call sensor.get(...), since there may be many rapid
   * calls to the gyro in rapid succession.
   */
  protected abstract void updateSensorCache();


  /**
   * @return the rate of yaw for the IMU. Units depend on the IMU implementation for getYaw()
   */
  public Rotation2d getYawRate() {
    return new Rotation2d(getYaw().minus(mLastYaw).getRadians() / dt);
  }

  /**
   * @return whether or not the current values of JerkX and JerkY constitute a collision
   */
  public boolean detectCollision(){
    // Combines both axes to get vector magnitude
    return Math.sqrt(Math.pow(mJerkX, 2) + Math.pow(mJerkY, 2)) >= mCollisionThreshold_DeltaG;
//    return Math.abs(mJerkX) >= mCollisionThreshold_DeltaG || Math.abs(mJerkY) >= mCollisionThreshold_DeltaG;
  }
  
  public double getFilteredAccelX() {
    return mAccelerationX.getAverage();
  }
  
  public double getFilteredAccelY() {
    return mAccelerationY.getAverage();
  }
  
  /**
   * @return the change in acceleration over time
   */
  public double getJerkX() {
    return mJerkX;
  }
  
  /**
   * @return the change in acceleration over time
   */
  public double getJerkY() {
    return mJerkY;
  }
  
  public Rotation2d getHeading() {
    return getYaw();
  }
}
