package us.ilite.robot.modules;

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
import us.ilite.common.config.*;
import us.ilite.common.types.sensor.EColorData;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.ICommand;

import static us.ilite.robot.utils.ColorUtils.*;

public class DJBoothPositionControl extends Module {

    private ColorSensorV3 mColorSensorV3;
    private final ColorMatch mColorMatcher = new ColorMatch();
    private EColorData.EColor eDesiredColorState;
    private EColorData.EColor eCurrentColorState;
    private EColorData.EColor eLastColorState;
    private EColorData.EInput eInputState;
    private int mSolidStateCounter;
    private EColorData.EMotorState eMotorState;
    private boolean mIsDone;
    private boolean mMightBeDone;

    public DJBoothPositionControl () {
        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);

        mSolidStateCounter = 0;
        eMotorState = EColorData.EMotorState.OFF;
        eCurrentColorState = EColorData.EColor.NONE;
        eLastColorState = EColorData.EColor.NONE;
        eDesiredColorState = EColorData.EColor.NONE;
        eInputState = EColorData.EInput.NEGATIVE;
        mIsDone = false;
        mMightBeDone = false;

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);
    }

    public EColorData.EMotorState update(double pNow) {

        if ( eInputState.equals(EColorData.EInput.POSITIVE) ) {
            if ( !mIsDone ) {
                updateColor();
                DriverStation.reportError( "Running Motor for Position Control", false );

                Color detectedColor = mColorSensorV3.getColor();
                ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
//                if ( !mMightBeDone ) {
                eLastColorState = eCurrentColorState;
//                }
                eCurrentColorState = getState( match.color );
                SmartDashboard.putString( "Detected Color on Rotation: ", getColorStringForMatchResult( match ) );

//                if ( (!eLastColorState.equals( ColorState.BLUE ) && eDesiredColorState.equals(ColorState.GREEN)) && !eCurrentColorState.equals(ColorState.GREEN)) {
//                    victorSPX.set(ControlMode.PercentOutput, Settings.kDJOutput );
//                    mSolidStateCounter = 0;
//                    mIsDone = false;
//                    mMightBeDone = false;
//                    return false;
//                }
//                else if ( (eLastColorState.equals( ColorState.BLUE ) && eDesiredColorState.equals(ColorState.GREEN)) && eCurrentColorState.equals(ColorState.GREEN)) {
//                    victorSPX.set(ControlMode.PercentOutput, 0d);
//                    mSolidStateCounter++;
//
//                    if ( mSolidStateCounter >= 5 ) {
//                        mIsDone = true;
//                        mMightBeDone = false;
//                        return true;
//                    }
//                    mIsDone = false;
//                    mMightBeDone = true;
//                    return false;
//                }
//                else if (  !eCurrentColorState.equals(eDesiredColorState) ) {
//                    victorSPX.set(ControlMode.PercentOutput, Settings.kDJOutput );
//                    mSolidStateCounter = 0;
//                    mIsDone = false;
//                    mMightBeDone = false;
//                    return false;
//                }
//                else {
                if (eCurrentColorState.equals(eDesiredColorState)) {
                    if ( mSolidStateCounter >= 5 ) {
                        mSolidStateCounter++;
                        mIsDone = true;
//                        mMightBeDone = false;
                        return EColorData.EMotorState.OFF;
                    }
                    else {
                        mSolidStateCounter++;
                        return EColorData.EMotorState.ON;
                    }
                }
            }
            mIsDone = true;
            return true;
        }
        return true;
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

    public EColorData.EColor getState(Color c ) {
        if ( c.equals( kBlueTarget ) ) {
            return EColorData.EColor.BLUE;
        }
        else if ( c.equals( kRedTarget ) ) {
            return EColorData.EColor.RED;
        }
        else if ( c.equals( kYellowTarget ) ) {
            return EColorData.EColor.YELLOW;
        }
        else if ( c.equals( kGreenTarget ) ) {
            return EColorData.EColor.GREEN;
        }
        else {
            return EColorData.EColor.NONE;
        }
    }

    public Color updateColor() {
        ColorMatchResult match = mColorMatcher.matchClosestColor( mColorSensorV3.getColor() );
        Data mData = Robot.DATA;
        if (match != null) {
            if (kBlueTarget.equals(match.color)) {
                mData.color.set( EColorData.SENSED_COLOR, (double)EColorData.EColor.BLUE.ordinal() );
                return kBlueTarget;
            } else if (kRedTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.RED.ordinal());
                return kRedTarget;
            } else if (kGreenTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.GREEN.ordinal());
                return kGreenTarget;
            } else if (kYellowTarget.equals(match.color)) {
                mData.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.YELLOW.ordinal());
                return kYellowTarget;
            } else {
                mData.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.NONE.ordinal());
                return null;
            }
        }
        return null;
    }

    public boolean isDone() {
        return mIsDone;
    }

    public void updateMotor( EColorData.EMotorState motorState ){
        eMotorState = motorState;
    }

    public void setDesiredColorState(EColorData.EColor pColorState ){
        eDesiredColorState = pColorState;
    }

    public void reset() {

    }

    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {

    }

    @Override
    public void shutdown(double pNow) {
        victorSPX.set(ControlMode.PercentOutput, 0d);
    }
}
