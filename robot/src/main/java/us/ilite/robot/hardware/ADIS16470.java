package us.ilite.robot.hardware;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class ADIS16470 extends IMU {
    private  ADIS16470_IMU imu = new ADIS16470_IMU();
    private Rotation2d
        yaw = ZERO,
        rate = ZERO,
        pitch = ZERO,
        roll = ZERO;
    private double
        rawAccelX = 0d,
        rawAccelY = 0d;


    public ADIS16470(Clock pClock) {
        super(pClock,DEFAULT_GAINS);
    }

    @Override
    protected void updateSensorCache() {
        yaw = Rotation2d.fromDegrees(imu.getAngle());
        rate = Rotation2d.fromDegrees(imu.getRate());
        rawAccelX = imu.getAccelInstantX();
        rawAccelY = imu.getAccelInstantY();
    }

    /**
     * Units are degrees
     * @return
     */
    @Override
    public Rotation2d getYaw() {
        return yaw;
    }

    @Override
    public Rotation2d getYawRate() {
        return rate;
    }

    @Override
    public Rotation2d getPitch() {
        return pitch;
    }

    @Override
    public Rotation2d getRoll() {
        return roll;
    }

    @Override
    public void zeroAll() {
        // unsupported
    }

    @Override
    protected double getRawAccelX() {
        return rawAccelX;
    }

    @Override
    protected double getRawAccelY() {
        return rawAccelY;
    }
}
