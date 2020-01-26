package us.ilite.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.sensor.EColorData;
import us.ilite.robot.Robot;

import static us.ilite.robot.utils.ColorUtils.*;

public class DJBoothPositionControl implements ICommand {

    private ColorSensorV3 mColorSensorV3;
    private VictorSPX victorSPX;
    private final ColorMatch mColorMatcher = new ColorMatch();
    private ColorState eDesiredColorState;
    private ColorState eCurrentColorState;
    private int mSolidStateCounter;
    private MotorState eMotorState;
    private  boolean mIsDone;

    public enum ColorState {
        DEFAULT,
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
        mSolidStateCounter = 0;
        eMotorState = MotorState.OFF;
        eDesiredColorState = ColorState.DEFAULT;
        mIsDone = false;

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);
    }

    @Override
    public boolean update(double pNow) {

        updateColor();

        if ( eMotorState.equals( MotorState.ON )) {
            DriverStation.reportError( "Running Motor for Position Control", false );

            Color detectedColor = mColorSensorV3.getColor();
            ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
            String colorString = getColorStringForMatchResult(match);
            eCurrentColorState = getState( match.color );
            SmartDashboard.putString( "Detected Color on Rotation: ", getColorStringForMatchResult( match ) );

            if ( !eCurrentColorState.equals(eDesiredColorState) ) {
                victorSPX.set(ControlMode.PercentOutput, Settings.kDJOutput );
                mSolidStateCounter = 0;
                mIsDone = false;
                return false;
            }
            else {
                victorSPX.set(ControlMode.PercentOutput, 0d);
                mSolidStateCounter++;
                if ( mSolidStateCounter >= 5 ) {
                    mIsDone = true;
                    return true;
                }
                mIsDone = false;
                return false;
            }
        }
        mIsDone = false;
        return false;
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

    public ColorState getState( Color c ) {
        if ( c.equals( kBlueTarget ) ) {
            return ColorState.BLUE;
        }
        else if ( c.equals( kRedTarget ) ) {
            return ColorState.RED;
        }
        else if ( c.equals( kYellowTarget ) ) {
            return ColorState.YELLOW;
        }
        else if ( c.equals( kGreenTarget ) ) {
            return ColorState.GREEN;
        }
        else {
            return ColorState.DEFAULT;
        }
    }

    public Color updateColor() {
        ColorMatchResult match = mColorMatcher.matchClosestColor( mColorSensorV3.getColor() );
        Data mData = Robot.DATA;
        if (match != null) {
            if (kBlueTarget.equals(match.color)) {
                mData.color.set( EColorData.SENSED_COLOR, EColorData.EColor.BLUE.ordinal() );
                return kBlueTarget;
            } else if (kRedTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, EColorData.EColor.RED.ordinal());
                return kRedTarget;
            } else if (kGreenTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, EColorData.EColor.GREEN.ordinal());
                return kGreenTarget;
            } else if (kYellowTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, EColorData.EColor.YELLOW.ordinal());
                return kYellowTarget;
            } else {
                mData.color.set(EColorData.SENSED_COLOR, Color.kBlack);
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

    public void setDesiredColorState( ColorState pColorState ){
        eDesiredColorState = pColorState;
    }

    @Override
    public void shutdown(double pNow) {
        victorSPX.set(ControlMode.PercentOutput, 0d);
    }
}
