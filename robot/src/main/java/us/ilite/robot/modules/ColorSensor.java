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
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private Data mData;

    public ColorSensor(Data mData) {
        this.mData = mData;
    }


    @Override
    public void modeInit(double pNow) {
        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);

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
        String colorString = "NULL MATCH";
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
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
        SmartDashboard.putString( "Detected Color: ", colorString );
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