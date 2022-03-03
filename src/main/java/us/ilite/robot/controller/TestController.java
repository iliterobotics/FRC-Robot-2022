package us.ilite.robot.controller;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EHangerModuleData;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.modules.LEDControl;

import static us.ilite.robot.Enums.*;
import static us.ilite.common.types.EFeederData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;

public class TestController extends BaseManualController {

    private ILog mLog = Logger.createLog(TestController.class);

    private Double mLastTargetTrackingType = 0d;
    private Double mLastGroundTrackingType = 0d;

    private double mLimelightZoomThreshold = 7.0;
    private double mLimelightGoalThreshold = 5.0;

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

        double spd = Math.max(db.drivetrain.get(R_ACTUAL_VEL_FT_s), db.drivetrain.get(L_ACTUAL_VEL_FT_s));
        mMaxSpeed = Math.max(mMaxSpeed, spd);
        SmartDashboard.putNumber("Max Robot Speed (ft/s)", mMaxSpeed);
//        SmartDashboard.putNumber("Max Drive RPM", mMaxSpeed / kDriveFalconVelocityFactor);

        mMaxYaw = Math.max(mMaxYaw, db.imu.get(EGyro.YAW_OMEGA_DEGREES));
        SmartDashboard.putNumber("Max Robot Omega (deg/s)", mMaxYaw);
    }

    public void updateClimber() {
        if (db.driverinput.isSet(InputMap.DRIVER.HANGER_EXECUTE)) {
            db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.POSITION);
            db.hanger.set(EHangerModuleData.L_DESIRED_POSITION_rot, 1);
            db.hanger.set(EHangerModuleData.R_DESIRED_POSITION_rot, 1);
        }
        else {
            db.hanger.set(EHangerModuleData.HANGER_STATE, Enums.EHangerMode.VELOCITY);
            db.hanger.set(EHangerModuleData.L_DESIRED_VEL_rpm, 0);
            db.hanger.set(EHangerModuleData.R_DESIRED_VEL_rpm, 0);
        }
    }

    public void updateLED() {
        if(db.driverinput.isSet(ELogitech310.X_BTN)) {
            db.ledcontrol.set(ELEDControlData.LED_STATE, 1.0);
            db.ledcontrol.set(ELEDControlData.DESIRED_R, 255);
            db.ledcontrol.set(ELEDControlData.DESIRED_G, 0);
            db.ledcontrol.set(ELEDControlData.DESIRED_B, 255);
        }
        else {
            db.ledcontrol.set(ELEDControlData.LED_STATE, 0.0);
        }
    }

}
