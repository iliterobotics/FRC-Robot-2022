package us.ilite.robot.hardware;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import us.ilite.common.types.EVisionGoal2020;

public class VisionGyro extends IMU {


    private RobotCodex<EVisionGoal2020> mTargetingData;
    private Rotation2d mGyroOffsetX = new Rotation2d();
    private Rotation2d mGyroOffsetY = new Rotation2d();

    public VisionGyro(Clock pClock, RobotCodex<EVisionGoal2020> pData) {
        super(pClock, DEFAULT_GAINS);
        mTargetingData = pData;
    }

    @Override
    public Rotation2d getYaw() {
        return getX().rotateBy(mGyroOffsetX);
    }

    @Override
    public Rotation2d getPitch() {
        return getY().rotateBy(mGyroOffsetY);
    }

    @Override
    public Rotation2d getRoll() {
        return ZERO;
    }

    @Override
    public void zeroAll() {
        mGyroOffsetX = getX();
        mGyroOffsetY = getY();
    }

    @Override
    protected double getRawAccelX() {
        return 0;
    }

    @Override
    protected double getRawAccelY() {
        return 0;
    }

    private Rotation2d getX() {
        Double x = mTargetingData.get(EVisionGoal2020.TX);

        if(x != null) {
            // Invert?
            return Rotation2d.fromDegrees(x);
        }
        
        return new Rotation2d();
    }

    private Rotation2d getY() {
        Double y = mTargetingData.get(EVisionGoal2020.TY);

        if(y != null) {
            return Rotation2d.fromDegrees(y);
        }
        
        return new Rotation2d();
    }

    @Override
    protected void updateSensorCache() {

    }



}
