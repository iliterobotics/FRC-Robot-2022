package us.ilite.robot.hardware;

import java.util.List;
import java.util.Collections;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import us.ilite.common.config.Settings;

public class Pigeon extends IMU{
	public static double kCollisionThreshold = 0.0;
	public static int kUpdateRate = 100;

	private double[] ypr;
	private short[] xyz;
	private double mLastYaw = 0;
	private double mYawRate = 0;
	private PigeonIMU mPigeon;

	/**
	 * Made this a singleton list because it is unmodifiable. When it was an array, it was possible for 
	 * the value to be changed. 
	 */
	//TODO - single value for now - could be VERY noisy
	// others to try: {0.75, 0.25}, {0.6, 0.4}, {0.5, 0.3, 0.2}
	private static List<Double>kCollisionGains = Collections.singletonList(Double.valueOf(1.0));
	
	public Pigeon(Clock pClock, int pPigeonCANId){
//	    mPigeon.
		super(pClock, kCollisionGains);
		ypr = new double[3];
		xyz = new short[3];
		this.mPigeon = new PigeonIMU(pPigeonCANId);
		ErrorCode ec = mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, (int)(1.0 / kUpdateRate * 1000.0), Settings.HW.CAN.kLongTimeoutMs);
		System.err.println("===== PIGEON ERROR CODE: " + ec);
		mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, (int)(1.0 / kUpdateRate * 1000.0), Settings.HW.CAN.kLongTimeoutMs);

		setCollisionThreshold_DeltaG(kCollisionThreshold);
		//mAccelerationX = new FilteredAverage(kCollisionGains);
		//mAccelerationY = new FilteredAverage(kCollisionGains);
	}

	public void resetAngle(Rotation2d pAngle) {
		mPigeon.setFusedHeading(pAngle.getDegrees(), 20);
	}
	
	/**
	 * Pre-populates the filters & calculated values so it's done only once per cycle
	 */
	protected void updateSensorCache() {
		mLastYaw = ypr[0];
		mPigeon.getYawPitchRoll(ypr);
		for(int i = 0 ; i < ypr.length; i++) {
		  ypr[i] = Rotation2d.fromDegrees(ypr[i]).getDegrees();
		}
		mPigeon.getBiasedAccelerometer(xyz);
	}
	
	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(mPigeon.getFusedHeading());
	}

	public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(ypr[0]);
	}
	
	public Rotation2d getPitch() {
    	return Rotation2d.fromDegrees(ypr[1]);
	}
	
	public Rotation2d getRoll() {
    	return Rotation2d.fromDegrees(ypr[2]);
	}	
	
	public double getAccelX() {
	  return mAccelerationX.getAverage();
	}
	
	public double getAccelY() {
	  return mAccelerationY.getAverage();
	}
	
	public double getJerkX() {
	  return mJerkX;
	}
	
	public double getJerkY() {
	  return mJerkY;
	}
	
	public void zeroAll() {
		for(int i = 0; i < ypr.length; i++) {
			ypr[i] = 0;
		}
		mPigeon.setYaw(0d, 20);
		mPigeon.setFusedHeading(0d, 20); //TODO - figure out CAN timeout defaults
	}

	public double getRawAccelX() {
		return xyz[0];
	}

	public double getRawAccelY() {
		return xyz[1];
	}
}