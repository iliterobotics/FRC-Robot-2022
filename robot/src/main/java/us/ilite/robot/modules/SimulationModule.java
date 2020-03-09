package us.ilite.robot.modules;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimulationModule extends Module {
    private double direction = 0d;
    private double lastAngle = 0d;
    private double currentAngle = 0d;
//    private Potentiometer mHoodPot;
    private double currentPositionDeg = 0d;
    private double currentVelocityDegPerSec = 0d;

    private static final double
            // REV is Speed: 0.14 s/60° (at 6V)
            MAX_SERVO_SPEED_DEG_PER_SEC = 60.0 / 0.14,
            MAX_SERVO_SPEED_SEC_PER_DEG = 1/MAX_SERVO_SPEED_DEG_PER_SEC,
            SERVO_LINEAR_INCH_PER_ROTATION = Math.PI*0.5,
            SERVO_MAX_LINEAR_SPEED_IN_PER_SEC = MAX_SERVO_SPEED_DEG_PER_SEC / 360.0 * SERVO_LINEAR_INCH_PER_ROTATION;
    private Servo mHoodServo;

    public SimulationModule() {
//        mHoodPot = new AnalogPotentiometer(0);
        mHoodServo = new Servo(0);
        SmartDashboard.putNumber("RAW POT", 0d);
    }

    @Override
    public void readInputs() {
        doPhysics();
        currentAngle = 0;
        currentPositionDeg += currentAngle - lastAngle;
        currentVelocityDegPerSec = (currentAngle - lastAngle) / clock.dt();
        direction = Math.signum(currentAngle - lastAngle);
    }
    public void setOutputs() {
        lastAngle = currentAngle;
        SmartDashboard.putNumber("RAW POT", currentAngle);
        SmartDashboard.putNumber("POT POS", currentPositionDeg);
        SmartDashboard.putNumber("POT VEL", currentVelocityDegPerSec);
        doPhysics();
    }

    private final void doPhysics() {

    }
}
