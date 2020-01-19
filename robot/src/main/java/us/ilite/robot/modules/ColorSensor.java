package us.ilite.robot.modules;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.Data;

import static us.ilite.robot.utils.ColorUtils.*;

public class ColorSensor extends Module{
    private ColorSensorV3 mColorSensorV3;
    private ColorSensorV3 mColorSensorV3Other;
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private Data mData;

    public ColorSensor(Data mData) {
        this.mData = mData;
    }


    @Override
    public void modeInit(double pNow) {
        I2C.Port i2cPorta = I2C.Port.kOnboard;
        //I2C.Port i2cPortb = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPorta);
        //mColorSensorV3Other = new ColorSensorV3( i2cPortb );

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    @Override
    public void periodicInput(double pNow) {

    }


    @Override
    public void update(double pNow) {
        Color detectedColor = mColorSensorV3.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        String colorString = getColorStringForMatchResult(match);
<<<<<<< HEAD
        SmartDashboard.putString( "Detected Color on A: ", colorString );

//        Color detectedColorOther = mColorSensorV3.getColor();
//        ColorMatchResult matchOther = m_colorMatcher.matchClosestColor(detectedColorOther);
//        String colorStringOther = getColorStringForMatchResult(matchOther);
//        SmartDashboard.putString( "Detected Color on B: ", colorStringOther );
=======
        SmartDashboard.putString( "Detected Color: ", colorString );
        SmartDashboard.putNumber("Red: ",detectedColor.red);
        SmartDashboard.putNumber("Green: ",detectedColor.green);
        SmartDashboard.putNumber("Blue: ",detectedColor.blue);
>>>>>>> e063a2da... Added Smartdashboard to RGB.
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

    public Color getColor() {
        Color detectedColor = mColorSensorV3.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        return match.color;
    }

    @Override
    public void shutdown(double pNow) {


    }
}