package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.Distance;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

import static us.ilite.common.types.drive.EDriveData.*;

public class DriveStraightController extends BaseAutonController {
    private Distance targetDistance;
    private Data db = Robot.DATA;
    private boolean isFirstLegDone;
    
    public DriveStraightController(Distance mDistance){
        db.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
        this.targetDistance = mDistance;
        isFirstLegDone = false;
    }

    @Override
    protected void updateImpl(double pNow) {
        super.updateImpl(pNow);
        db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
        if (isAtDistance(targetDistance)){
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0.5);
            isFirstLegDone = true;
        }
        activateSerializer(pNow);
        setIntakeArmEnabled(pNow, true);
        initiateAllModules(isFirstLegDone);
    }

    public Distance getDistance() {
        return Distance.fromFeet(
                (Robot.DATA.drivetrain.get(L_ACTUAL_POS_FT) +
                        Robot.DATA.drivetrain.get(R_ACTUAL_POS_FT)) / 2.0);
    }

    public boolean isAtDistance(Distance targetDistance){
        return getDistance().inches() >= targetDistance.inches();
    }

    public void initiateAllModules(boolean isFirstLegDone){
        if(isFirstLegDone){
            //TODO implement flywheel module stuff

        }
    }
}
