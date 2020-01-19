package us.ilite.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.Settings;
import us.ilite.robot.utils.ColorUtils;

import static us.ilite.robot.utils.ColorUtils.*;

public class DJBoothRotationControl implements ICommand {

    private ColorSensorV3 mColorSensorV3;
    private VictorSPX victorSPX;

    private final ColorMatch mColorMatcher = new ColorMatch();
    private ColorState eCurrentColorState;
    private ColorState eLastColorState;
    private int mColorChangeLocation;
    private int mInitialColorStateLocation;
    private final ColorState[] colorStates = {ColorState.RED, ColorState.YELLOW, ColorState.BLUE, ColorState.GREEN};
    private ILog mLog = Logger.createLog( this.getClass() );
    private MotorState eMotorState;
    private boolean mIsDone;

    public enum ColorState {
        RED,
        GREEN,
        BLUE,
        YELLOW;
    }

    public enum MotorState {
        ON,
        OFF;
    }

    @Override
    public void init(double pNow) {
        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);
        victorSPX = new VictorSPX( 12 );
        victorSPX.set( ControlMode.PercentOutput, 0d );

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);

        setColorState( mColorMatcher.matchClosestColor( mColorSensorV3.getColor() ).color );
        eLastColorState = eCurrentColorState;
        mColorChangeLocation = getInitialStateLocation( (ColorState) eCurrentColorState );
    }

    @Override
    public boolean update(double pNow) {

        if ( (mColorChangeLocation - mInitialColorStateLocation <= 32) && eMotorState == MotorState.ON ) {
            Color mColor = mColorSensorV3.getColor();
            eLastColorState = eCurrentColorState;
            ColorMatchResult match = mColorMatcher.matchClosestColor( mColor );
            setColorState( match.color );

//        String colorString = getColorStringForMatchResult(match);
//        SmartDashboard.putString( "Detected Color: ", colorString );
            if ( eCurrentColorState != eLastColorState ) {
                mColorChangeLocation++;
            }
            if ( colorStates[ mColorChangeLocation % colorStates.length ] != eCurrentColorState ) {
                mLog.warn( "Issue with Detecting Color. Actual: " + colorStates[mColorChangeLocation] + "Detected: " + eLastColorState );
                mColorChangeLocation--;
                eCurrentColorState = getStateAtLocation( mColorChangeLocation );
            }
//            String colorString = getColorStringForMatchResult(match);
//            SmartDashboard.putString( "Detected Color on Rotation: ", getColorStringForMatchResult( match ) );

            victorSPX.set(ControlMode.PercentOutput, Settings.kDJOutput );
            mIsDone = false;
            return false;
        }
        else if ( eMotorState == MotorState.ON ) {
            mIsDone = true;
            victorSPX.set(ControlMode.PercentOutput, 0d);
            return true;
        }
        else {
            mIsDone = false;
            victorSPX.set(ControlMode.PercentOutput, 0d);
            return false;
        }
    }

    protected static String getColorStringForMatchResult(ColorMatchResult match) {
        String colorString = "NULL MATCH";
        if(match != null) {
            if (kBlueTarget.equals(match.color)) {
                colorString = "Blue";
            } else if (kRedTarget.equals(match.color)) {
                colorString = "Red";
            } else if (kGreenTarget.equals(match.color)) {
                colorString = "Green";
            } else if (kYellowTarget.equals(match.color)) {
                colorString = "Yellow";
            } else {
                colorString =
                        "Unknown: " + ((match.color != null) ?
                                "RGB={" + match.color.red + ", " + match.color.green + ", " + match.color.blue + "}" : "null");
            }
        }
        return colorString;
    }

    public boolean setColorState( Color color ) {
        if (kBlueTarget.equals(color)) {
            eCurrentColorState = ColorState.BLUE;
        } else if (kRedTarget.equals(color)) {
            eCurrentColorState = ColorState.RED;
        } else if (kGreenTarget.equals(color)) {
            eCurrentColorState = ColorState.GREEN;
        } else if (kYellowTarget.equals(color)) {
            eCurrentColorState = ColorState.YELLOW;
        }
        else {
            return false;
        }
        return true;
    }

    private int getInitialStateLocation( ColorState colorState ){
        switch ( colorState ) {
            case RED: return 0;
            case YELLOW: return 1;
            case BLUE: return 2;
            case GREEN: return 3;
            default: return -1;
        }
    }

    private ColorState getStateAtLocation( int i ) {
        switch ( i % colorStates.length ) {
            case 0: return ColorState.RED;
            case 1: return ColorState.YELLOW;
            case 2: return ColorState.BLUE;
            case 3: return ColorState.GREEN;
            default: return null;
        }
    }

    public Color getColor() {
        ColorMatchResult match = mColorMatcher.matchClosestColor( mColorSensorV3.getColor() );
        if (match != null) {
            if (kBlueTarget.equals(match.color)) {
                return kBlueTarget;
            } else if (kRedTarget.equals(match.color)) {
                return kRedTarget;
            } else if (kGreenTarget.equals(match.color)) {
                return kGreenTarget;
            } else if (kYellowTarget.equals(match.color)) {
                return kYellowTarget;
            } else {
                return null;
            }
        }
        return null;
    }

    public boolean isDone() {
        return mIsDone;
    }

    public void updateMotor( MotorState motorState ){
        eMotorState = motorState;
    }

    @Override
    public void shutdown(double pNow) {
        victorSPX.set(ControlMode.PercentOutput, 0d);
    }
}