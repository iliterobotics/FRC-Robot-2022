package us.ilite.robot.modules;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import edu.wpi.first.wpilibj.Joystick;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.DJBoothPositionControl;
import us.ilite.robot.commands.DJBoothRotationControl;

public class OperatorInput extends Module {
    protected static final double
            DRIVER_SUB_WARP_AXIS_THRESHOLD = 0.5;
    private ILog mLog = Logger.createLog(OperatorInput.class);


    private Joystick mDriverJoystick;
    private Joystick mOperatorJoystick;

    private DJBoothPositionControl mDjBoothPositionControl;
    private DJBoothRotationControl mDjBoothRotationControl;

    protected Codex<Double, ELogitech310> mDriverInputCodex, mOperatorInputCodex;

    public OperatorInput( DJBoothRotationControl pDjBoothRotationControl, DJBoothPositionControl pDjBoothPositionControl) {
        mDriverJoystick = new Joystick(0);
        mOperatorJoystick = new Joystick(1);

        this.mDjBoothPositionControl = pDjBoothPositionControl;
        this.mDjBoothRotationControl = pDjBoothRotationControl;
    }

    @Override
    public void modeInit(EMatchMode pMode, double pNow) {

    }

    @Override
    public void readInputs(double pNow) {
        ELogitech310.map(Robot.DATA.driverinput, mDriverJoystick);
//        ELogitech310.map(Robot.mData.operatorinput, mOperatorJoystick);

        updateDJBooth();
    }

    private void updateDJBooth() {
        if ( mDriverInputCodex.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL) &&
                mDriverInputCodex.isSet(InputMap.OPERATOR.OPERATOR_ROTATION_CONTROL) ) {
            mDjBoothPositionControl.updateMotor( DJBoothPositionControl.MotorState.OFF );
            mDjBoothRotationControl.updateMotor( DJBoothRotationControl.MotorState.OFF );
        }
        else if (mDriverInputCodex.isSet(InputMap.OPERATOR.OPERATOR_POSITION_CONTROL)) {
            mDjBoothPositionControl.updateMotor( DJBoothPositionControl.MotorState.ON );
            mDjBoothPositionControl.setDesiredColorState( DJBoothPositionControl.ColorState.RED );
        }
        else if (mDriverInputCodex.isSet(InputMap.OPERATOR.OPERATOR_ROTATION_CONTROL) ) {
            mDjBoothRotationControl.updateMotor( DJBoothRotationControl.MotorState.ON );
        }
        else {
            mDjBoothPositionControl.updateMotor(DJBoothPositionControl.MotorState.OFF);
            mDjBoothRotationControl.updateMotor(DJBoothRotationControl.MotorState.OFF);
        }
    }

    @Override
    public void setOutputs(double pNow) {
    }


    @Override
    public void shutdown(double pNow) {

    }



}
