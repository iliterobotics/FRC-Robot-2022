package us.ilite.robot.modules;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.Data;

import java.awt.*;

public class ColorSensor extends Module{
    private ColorSensorV3 mColorSensorV3;
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private Data mData;

    public ColorSensor(Data mData) {
        this.mData = mData;
    }


    @Override
    public void modeInit(double pNow) {
        I2C.Port i2cPort = I2C.Port.kOnboard;
        ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cPort);

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

        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
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



