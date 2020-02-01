package us.ilite.common.types.input;

import com.flybotix.hfr.codex.Codex;
import com.flybotix.hfr.codex.CodexOf;
import edu.wpi.first.wpilibj.Joystick;

public enum ELogitechAttack3 implements CodexOf<Double> {
    // DO NOT CHANGE THE ORDER OF THE BUTTONS
    TRIGGER,
    THUMB_DOWN,
    THUMB_CENTER,
    THUMB_LEFT,
    THUMB_RIGHT,
    BTN6,
    BTN7,
    BTN8,
    BTN9,
    BTN10,
    BTN11,
    X_AXIS,
    Y_AXIS,
    Z_AXIS;


    public static void map(Codex<Double, ELogitechAttack3> pCodex, Joystick pJoystick) {
        map(pCodex, pJoystick, null);
    }

    public static void map(Codex<Double, ELogitechAttack3> pCodex, Joystick pJoystick, Double pRumbleValue, boolean pHandleDeadband) {
        pCodex.reset();
        for(int i = 0 ; i < 10; i++) {
            pCodex.set(i, pJoystick.getRawButton(i+1) ? 1d : null);
        }
        if (pHandleDeadband) {
            pCodex.set(ELogitechAttack3.X_AXIS, DriverInputUtils.handleDeadband(pJoystick, 0));
            pCodex.set(ELogitechAttack3.Y_AXIS, DriverInputUtils.handleDeadband(pJoystick, 1));
            pCodex.set(ELogitechAttack3.Z_AXIS, DriverInputUtils.handleDeadband(pJoystick, 2));
        }
        else {
            pCodex.set(ELogitechAttack3.X_AXIS, pJoystick.getRawAxis(0));
            pCodex.set(ELogitechAttack3.Y_AXIS, pJoystick.getRawAxis(1));
            pCodex.set(ELogitechAttack3.Z_AXIS, pJoystick.getRawAxis(2));
        }
    }

    public static void map(Codex<Double, ELogitechAttack3> pCodex, Joystick pJoystick, Double pRumbleValue) {
        map(pCodex, pJoystick, pRumbleValue, false);
    }

}
