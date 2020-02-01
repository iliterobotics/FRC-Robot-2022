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

    public static ColorSensorV3 mColorSensorV3;
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

    public EColorData.EMotorState update() {

        if ( eInputState.equals(EColorData.EInput.POSITIVE) ) {
            if ( !mIsDone ) {
                DriverStation.reportError( "Running Motor for Position Control", false );

                Color detectedColor = getColor( Robot.DATA.color.get( EColorData.SENSED_COLOR ) );
                ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
//                if ( !mMightBeDone ) {
                eLastColorState = eCurrentColorState;
//                }
                eCurrentColorState = getState( match.color );
//                SmartDashboard.putString( "Detected Color on Rotation: ", getColorStringForMatchResult( match ) );

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
                mSolidStateCounter = 0;
            }
            mIsDone = true;
            return EColorData.EMotorState.OFF;
        }
        reset();
        return EColorData.EMotorState.OFF;
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

    public Color getColor( EColorData.EColor detectedColor ) {
        if ( detectedColor != null ) {
            if ( detectedColor.equals( EColorData.EColor.BLUE ) ) {
                return kBlueTarget;
            } else if ( detectedColor.equals( EColorData.EColor.RED ) ) {
                return kRedTarget;
            } else if ( detectedColor.equals( EColorData.EColor.GREEN ) ) {
                return kGreenTarget;
            } else if ( detectedColor.equals( EColorData.EColor.YELLOW ) ) {
                return kYellowTarget;
            }
        }
        return Color.kBlack;
    }

    public Color updateColor() {
        ColorMatchResult match = mColorMatcher.matchClosestColor( mColorSensorV3.getColor() );
        if (match != null) {
            if (kBlueTarget.equals(match.color)) {
                db.color.set( EColorData.SENSED_COLOR, (double)EColorData.EColor.BLUE.ordinal() );
                return kBlueTarget;
            } else if (kRedTarget.equals(match.color)) {
                db.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.RED.ordinal());
                return kRedTarget;
            } else if (kGreenTarget.equals(match.color)) {
                db.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.GREEN.ordinal());
                return kGreenTarget;
            } else if (kYellowTarget.equals(match.color)) {
                db.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.YELLOW.ordinal());
                return kYellowTarget;
            } else {
                db.color.set(EColorData.SENSED_COLOR, (double)EColorData.EColor.NONE.ordinal());
                return null;
            }
        }
        return null;
    }

    public void reset() {
        mSolidStateCounter = 0;
        eMotorState = EColorData.EMotorState.OFF;
        eCurrentColorState = EColorData.EColor.NONE;
        eLastColorState = EColorData.EColor.NONE;
        eDesiredColorState = EColorData.EColor.NONE;
        eInputState = EColorData.EInput.NEGATIVE;
        mIsDone = false;
        mMightBeDone = false;
    }

    @Override
    public void readInputs(double pNow) {
        if ( db.color.get(EColorData.POSITION_CONTROL_INPUT).equals((double)EColorData.EInput.POSITIVE.ordinal()) ) {
            eInputState = EColorData.EInput.POSITIVE;
        }
        else {
            eInputState = EColorData.EInput.NEGATIVE;
        }
    }

    @Override
    public void setOutputs(double pNow) {
        updateColor();
        db.color.set(EColorData.COLOR_WHEEL_MOTOR_STATE, update() );
        if (mIsDone) {
            db.color.set(EColorData.FINISHED, EColorData.EIsFinished.YES );
        }
        else {
            db.color.set(EColorData.FINISHED, EColorData.EIsFinished.YES );
        }
    }

}
