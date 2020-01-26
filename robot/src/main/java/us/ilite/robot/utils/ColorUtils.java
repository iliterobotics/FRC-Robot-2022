package us.ilite.robot.utils;

import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

public class ColorUtils {
    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


    /**Private class so this class cannot be instantiated since it's a utilities**/
    private ColorUtils() {}

}
