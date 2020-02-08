package us.ilite.common.types.input;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexOf;
import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.Joystick;
import us.ilite.common.Data;

import java.awt.*;


public enum ELogitech310 implements CodexOf<Double> {
    // Do not change the order of these buttons
    A_BTN,
    B_BTN,
	X_BTN,
	Y_BTN,
	L_BTN,
	R_BTN,
    BACK,
    START,
	LEFT_JOYSTICK_BTN,
	RIGHT_JOYSTICK_BTN,
	DPAD_RAW,
	DPAD_UP,
	DPAD_DOWN,
	DPAD_LEFT,
	DPAD_RIGHT,
	LEFT_X_AXIS,
	LEFT_Y_AXIS,
    RIGHT_X_AXIS,
    RIGHT_Y_AXIS,
    COMBINED_TRIGGER_AXIS,
    LEFT_TRIGGER_AXIS,
    RIGHT_TRIGGER_AXIS,
    RUMBLE;

	public static void map(RobotCodex<ELogitech310> pCodex, Joystick pJoystick) {
		map(pCodex, pJoystick, 0d);
	}
  
    public static void map(RobotCodex<ELogitech310> pCodex, Joystick pJoystick, double pRumbleValue, boolean pHandleDeadband) {
        pCodex.reset();
        for(int i = 0 ; i < 10; i++) {
          pCodex.set(i, pJoystick.getRawButton(i+1) ? 1d : Data.NULL_CODEX_VALUE);
        }

        if(pHandleDeadband) {
            pCodex.set(LEFT_X_AXIS, DriverInputUtils.handleDeadband(pJoystick, 0));
            pCodex.set(LEFT_Y_AXIS, DriverInputUtils.handleDeadband(pJoystick, 1));
            pCodex.set(LEFT_TRIGGER_AXIS,  DriverInputUtils.handleDeadband(pJoystick, 2));
            pCodex.set(RIGHT_TRIGGER_AXIS,  DriverInputUtils.handleDeadband(pJoystick, 3));
            pCodex.set(COMBINED_TRIGGER_AXIS, DriverInputUtils.handleDeadbandOfDifference(pJoystick, 2, 3));
            pCodex.set(RIGHT_X_AXIS, DriverInputUtils.handleDeadband(pJoystick, 4));
            pCodex.set(RIGHT_Y_AXIS, DriverInputUtils.handleDeadband(pJoystick, 5));
        } else {
            pCodex.set(LEFT_X_AXIS, pJoystick.getRawAxis(0));
            pCodex.set(LEFT_Y_AXIS, pJoystick.getRawAxis(1));
            pCodex.set(LEFT_TRIGGER_AXIS, pJoystick.getRawAxis(2));
            pCodex.set(RIGHT_TRIGGER_AXIS, pJoystick.getRawAxis(3));
            pCodex.set(COMBINED_TRIGGER_AXIS, pJoystick.getRawAxis(3) - pJoystick.getRawAxis(2));
            pCodex.set(RIGHT_X_AXIS, pJoystick.getRawAxis(4));
            pCodex.set(RIGHT_Y_AXIS, pJoystick.getRawAxis(5));
        }
        pCodex.set(RUMBLE, pRumbleValue);

        double dpad = (double)pJoystick.getPOV(0);
        if(dpad >= 0) {
            pCodex.set(DPAD_RAW, dpad);
            pCodex.set(DPAD_UP, dpad > 315 || dpad < 45 ? 1d : null);
            pCodex.set(DPAD_RIGHT, dpad > 45 && dpad < 135 ? 1d : null);
            pCodex.set(DPAD_DOWN, dpad > 135 && dpad < 225 ? 1d : null);
            pCodex.set(DPAD_LEFT, dpad > 225 && dpad < 315 ? 1d : null);
        }
    }
	
	public static void map(RobotCodex<ELogitech310> pCodex, Joystick pJoystick, double pRumbleValue) {
		map(pCodex, pJoystick, pRumbleValue, false);
	}

	public boolean isAxis() {
	    switch(this) {
            case LEFT_X_AXIS:
            case LEFT_Y_AXIS:
            case RIGHT_X_AXIS:
            case RIGHT_Y_AXIS:
            case COMBINED_TRIGGER_AXIS:
            case LEFT_TRIGGER_AXIS:
            case RIGHT_TRIGGER_AXIS:
            case RUMBLE:
            return true;
        }
        return false;
    }

    public boolean isButton() {
        switch(this) {
            case A_BTN:
            case B_BTN:
            case X_BTN:
            case Y_BTN:
            case L_BTN:
            case R_BTN:
            case BACK:
            case START:
            case LEFT_JOYSTICK_BTN:
            case RIGHT_JOYSTICK_BTN:
            case DPAD_RAW:
            case DPAD_UP:
            case DPAD_DOWN:
            case DPAD_LEFT:
            case DPAD_RIGHT:
            return true;
        }
        return false;
    }
}
