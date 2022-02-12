package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EGyro;
import static us.ilite.robot.Enums.*;
import static us.ilite.common.types.EShooterSystemData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;

public class TestController extends BaseManualController {

    private ILog mLog = Logger.createLog(TestController.class);

    private Double mLastTargetTrackingType = 0d;
    private Double mLastGroundTrackingType = 0d;

    private double mLimelightZoomThreshold = 7.0;
    private double mLimelightGoalThreshold = 5.0;

    private EIntakeState mIntakeState;
    private double mPreviousTime;
    private double mGoalBeamCountBroken = 0;
    private boolean crossedEntry = false;
    private Boolean allBeamsBroken = false;

    private static TestController INSTANCE;

    public static TestController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TestController();
        }
        return INSTANCE;
    }

    private TestController() {
        db.registerAllWithShuffleboard();
    }

    private double mMaxSpeed = 0.0;
    private double mMaxYaw = 0.0;


    @Override
    protected void updateImpl() {
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        clock.report("updateDrivetrain", t -> updateDrivetrain(true));

        double spd = Math.max(db.drivetrain.get(R_ACTUAL_VEL_FT_s), db.drivetrain.get(L_ACTUAL_VEL_FT_s));
        mMaxSpeed = Math.max(mMaxSpeed, spd);
        SmartDashboard.putNumber("Max Robot Speed (ft/s)", mMaxSpeed);
//        SmartDashboard.putNumber("Max Drive RPM", mMaxSpeed / kDriveFalconVelocityFactor);

        mMaxYaw = Math.max(mMaxYaw, db.imu.get(EGyro.YAW_OMEGA_DEGREES));
        SmartDashboard.putNumber("Max Robot Omega (deg/s)", mMaxYaw);
    }

    public void updateHanger() {
        if(db.driverinput.isSet(ELogitech310.R_BTN) && db.operatorinput.isSet(ELogitech310.R_BTN)) {
            db.hanger.set(EHangerModuleData.SET_pct, 0.3);
        }
        else if (db.driverinput.isSet(ELogitech310.L_BTN) && db.operatorinput.isSet(ELogitech310.L_BTN)) {
            db.hanger.set(EHangerModuleData.SET_pct, -0.3);
        }
        else {
            db.hanger.set(EHangerModuleData.SET_pct, 0);
        }
    }

}

