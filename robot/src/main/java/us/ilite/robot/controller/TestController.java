package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.types.EShooterData;
import us.ilite.common.types.ETargetingData;
import us.ilite.robot.modules.FlywheelModule;

public class TestController extends AbstractController {

    private FlywheelModule mShooter;
    private Data mData;

    public TestController(Data pData, FlywheelModule pShooter) {
        mData = pData;
        mShooter = pShooter;
    }

    private void updateShooter(double pTime) {
        if ( mData.flywheel.get(EShooterData.CURRENT_FLYWHEEL_STATE).equals((double)FlywheelModule.EShooterState.SHOOT.ordinal()) ) {
            mShooter.hoodAngle();
            mShooter.shoot(pTime);
        }
        if ( mShooter.isGyro ) {
            mShooter.turretTurn(true);
        }
        else if ( mData.limelight.get(ETargetingData.tx) != null ){
            mShooter.turretTurn(false);
        }
    }
    public void update(double pNow) {
        updateShooter(pNow);
    }
}
