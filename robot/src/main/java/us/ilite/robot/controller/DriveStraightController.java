package us.ilite.robot.controller;

import us.ilite.common.Data;
import us.ilite.common.Distance;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;

import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_POS_FT;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_POS_FT;

public class DriveStraightController extends BaseAutonController {
    private Distance targetDistance;
    private Data db = Robot.DATA;
    private boolean isFirstLegDone;
    
    public DriveStraightController(Distance mDistance){
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
        initiateAllModules(isFirstLegDone);
    }
    public Distance getDistance() {
        return Distance.fromFeet(
                (Robot.DATA.drivetrain.get(L_ACTUAL_POS_FT) +
                        Robot.DATA.drivetrain.get(R_ACTUAL_POS_FT)) / 2.0);
    }
    public boolean isAtDistance(Distance targetDistance){
        if (getDistance().inches() >= targetDistance.inches()){
            return true;
        }
        return false;
    }
    public void initiateAllModules(boolean isFirstLegDone){
        if(isFirstLegDone){
            db.powercell.set(EPowerCellData.INTAKE_STATE , 0.5);
            //TODO implement flywheel module stuff
            db.powercell.set(EPowerCellData.DESIRED_H_VELOCITY , 0.5);
            db.powercell.set(EPowerCellData.DESIRED_V_VELOCITY , 0.5);

        }
    }
}
