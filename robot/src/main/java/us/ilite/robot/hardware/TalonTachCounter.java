package us.ilite.robot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.loops.Loop;
import us.ilite.robot.modules.Module;

/**
 * Rapidly polls the Talon Tach for readings.
 * As the Talon Tach is designed for reading velocities, it isn't recommended that you use this - it's pretty likely that
 * you won't be able to poll fast enough and will miss a rising or falling edge.
 */
public class TalonTachCounter extends Module {

    private DigitalInput talonTachSensor;
    private boolean mCurrentState, lastState, hasChanged;
    private TapeState lastTapeState, mCurrentTapeState;

    public TalonTachCounter(int port)
    {
        talonTachSensor = new DigitalInput(port);
        mCurrentState = true;
        lastState = true;
        hasChanged = false;
        mCurrentTapeState = TapeState.NON_TAPE;
        lastTapeState = TapeState.NON_TAPE;
    }

    public enum TapeState
    {
        TAPE,
        NON_TAPE;
    }

    //true for reflective surfaces (powdercoat) false for non-reflective (tape)
    public boolean getSensor()
    {
        boolean actualHasChanged = hasChanged;
        hasChanged = false;
        return actualHasChanged;
    }

//	?public boolean getSensor() {
//    if(talonTachSensor == null) {
//      System.err.println("talon tach is null...");
//      return false;
//    } else {
//      return talonTachSensor.get();
//    }
//  }

    public boolean getState() {
        if(talonTachSensor == null) {
            System.err.println("talon tach is null...");
            return false;
        } else {
            return talonTachSensor.get();
        }
    }

    @Override
    public void modeInit(EMatchMode pMode) {
        // TODO Auto-generated method stub

    }

    @Override
    public void readInputs() {

    }

    public void setTapeState(boolean pCurrentState)
    {
        if(pCurrentState == false && lastState == true) mCurrentTapeState = TapeState.TAPE;
        if(pCurrentState == true && lastState == false) mCurrentTapeState = TapeState.NON_TAPE;
    }


    public TapeState getTapeState()
    {
        return mCurrentTapeState;
    }

    @Override
    public void setOutputs() {
        mCurrentState = getState();
        setTapeState(mCurrentState);
        if(mCurrentTapeState != lastTapeState) hasChanged = true;

        lastTapeState = mCurrentTapeState;
        lastState = mCurrentState;
    }

    @Override
    public void shutdown() {
        // TODO Auto-generated method stub

    }

}
