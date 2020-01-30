package us.ilite.robot;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.ETrackingType;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.EInputScale;
import us.ilite.common.types.input.ELogitech310;
import static us.ilite.robot.hardware.ECommonControlMode.*;

import us.ilite.robot.modules.Module;
import us.ilite.robot.modules.*;

@Deprecated
public class DriverInput extends Module implements IThrottleProvider, ITurnProvider {

    protected static final double
            DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(DriverInput.class);


    protected DriveModule mDrive;
    private CommandManager mTeleopCommandManager;
    private CommandManager mAutonomousCommandManager;
    private Limelight mLimelight;
    private Data mData;
    private Timer mGroundCargoTimer = new Timer();

    private boolean mIsCargo = false;
    private Joystick mDriverJoystick;
    private Joystick mOperatorJoystick;

    protected Codex<Double, ELogitech310> mDriverInputCodex, mOperatorInputCodex;

    private ETrackingType mLastTrackingType = null;

    public DriverInput(boolean pSimulated) {
//        this.mTeleopCommandManager = pTeleopCommandManager;
//        this.mAutonomousCommandManager = pAutonomousCommandManager;

        this.mDriverInputCodex = Robot.DATA.driverinput;
        this.mOperatorInputCodex = Robot.DATA.operatorinput;
        if(pSimulated) {
            // Use a different joystick library?

        } else {
            this.mDriverJoystick = new Joystick(0);
            this.mOperatorJoystick = new Joystick(1);
        }
    }


    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        ELogitech310.map(Robot.DATA.driverinput, mDriverJoystick);
        ELogitech310.map(Robot.DATA.operatorinput, mOperatorJoystick);
    }

    @Override
    public void setOutputs(double pNow) {
        /*
        If the driver started the commands that the superstructure is running and then released the button,
        stop running commands.
        */
        updateDriveTrain();


    }

    private void updateDriveTrain() {
        double rotate = getTurn();
        double throttle = getThrottle();

        rotate = Math.abs(rotate) > 0.01 ? rotate : 0.0; //Handling Deadband
        throttle = Math.abs(throttle) > 0.01 ? throttle : 0.0; //Handling Deadband

        if (throttle == 0.0 && rotate != 0.0) {
            throttle += 0.01;
        }
        //		    throttle = EInputScale.EXPONENTIAL.map(throttle, 2);
        rotate = EInputScale.EXPONENTIAL.map(rotate, 2);
        rotate *= Settings.Input.kNormalPercentThrottleReduction;

        if (Robot.DATA.driverinput.isSet(InputMap.DRIVER.SUB_WARP_AXIS) && Robot.DATA.driverinput.get(InputMap.DRIVER.SUB_WARP_AXIS) > DRIVER_SUB_WARP_AXIS_THRESHOLD) {
            throttle *= Settings.Input.kSnailModePercentThrottleReduction;
            rotate *= Settings.Input.kSnailModePercentRotateReduction;
        }

        Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN, rotate);
        Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE, throttle);

//        DriveMessage driveMessage = new DriveMessage().throttle(throttle).turn(rotate).mode(PERCENT_OUTPUT).normalize();//.calculateCurvature();
//        double leftSetpoint = driveMessage.getLeftOutput();
//        double rightSetpoint = driveMessage.getRightOutput();
//        leftSetpoint = Math.abs(leftSetpoint) > 0.01 ? leftSetpoint : 0.0; //Handling Deadband
//        rightSetpoint = Math.abs(rightSetpoint) > 0.01 ? rightSetpoint : 0.0; //Handling Deadband
//        Robot.DATA.drivetrain.set(EDriveData.LEFT_DEMAND, leftSetpoint * Settings.Drive.kDriveTrainMaxVelocity);
//        Robot.DATA.drivetrain.set(EDriveData.RIGHT_DEMAND, rightSetpoint * Settings.Drive.kDriveTrainMaxVelocity);



    }

    @Override
    public void shutdown(double pNow) {

    }

    @Override
    public double getThrottle() {
        if(Robot.DATA.driverinput.isSet(InputMap.DRIVER.THROTTLE_AXIS)) {
            return -Robot.DATA.driverinput.get(InputMap.DRIVER.THROTTLE_AXIS);
        } else {
            return 0.0;
        }
    }

    @Override
    public double getTurn() {
        if(Robot.DATA.driverinput.isSet(InputMap.DRIVER.TURN_AXIS)) {
            return Robot.DATA.driverinput.get(InputMap.DRIVER.TURN_AXIS);
        } else {
            return 0.0;
        }
    }
}
