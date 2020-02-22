package us.ilite.robot.controller;

import us.ilite.common.Angle;
import us.ilite.common.Data;
import us.ilite.common.Distance;
import us.ilite.common.Field2020;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EPowerCellData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;

import static us.ilite.common.types.drive.EDriveData.*;

public class DriveStraightController extends BaseAutonController {
    private Distance targetDistance;
    private Data db = Robot.DATA;
    private boolean isFirstLegDone;

    public DriveStraightController(){
        db.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
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
        initiateAllModules(pNow , isFirstLegDone);
    }

    public Distance getDistance() {
        return Distance.fromFeet(
                (Robot.DATA.drivetrain.get(L_ACTUAL_POS_FT) +
                        Robot.DATA.drivetrain.get(R_ACTUAL_POS_FT)) / 2.0);
    }

    public boolean isAtDistance(Distance targetDistance){
        return getDistance().inches() >= targetDistance.inches();
    }

    public void initiateAllModules(double pNow , boolean isFirstLegDone){
        if (isFirstLegDone){
            if (isFeederUpToSpeed() && isFlywheelUpToSpeed() &&
                    Field2020.canHitInnerGoal(tempCalcAngleToInnerGoal() , getDistance())){
                //TODO add in flywheel logic

            }
        }
    }

    public Angle tempCalcAngleToInnerGoal() {
        double thetaGoal = db.limelight.get(ELimelightData.CALC_ANGLE_TO_TARGET);
        double distToGoal = db.limelight.get(ELimelightData.CALC_DIST_TO_TARGET); //TODO: Units??
        double b = 29.25 + (distToGoal * Math.cos(thetaGoal));
        double a = distToGoal * Math.sin(thetaGoal);
        double thetab = Math.tanh(b/a);
        return Angle.fromDegrees(90 - thetab);
    }
}
