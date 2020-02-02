package us.ilite.robot.modules;

import com.flybotix.hfr.codex.Codex;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import us.ilite.common.types.ELimelightData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.targetData.ITargetDataProvider;

import java.util.Optional;

import static us.ilite.common.types.ELimelightData.CALC_ANGLE_TO_TARGET;

public class RawLimelight extends Module implements ITargetDataProvider {

    public static String tableName = "groundlimelight";
    // =============================================================================
    // LimeLight Camera Constants
    // Note: These constants need to be recalculated for a specific robot geometry
    // =============================================================================
    public static double kHeightIn = 58.0;
    public static double kToBumperIn = 10.0;
    public static double kAngleDeg = 28.55;

    public static double llFOVVertical = 49.7;
    public static double llFOVHorizontal = 59.6;

    // Left angle coefficients for angle = a + bx + cx^2
    //    a	0.856905324060421
    //    b	-3.01414088331715
    //    c	-0.0331854848038372
    public static double kLeftACoeff = 0.856905324060421;
    public static double kLeftBCoeff = -3.01414088331715;
    public static double kLeftCCoeff = -0.0331854848038372;

    // Right angle coefficients for angle = a + bx + cx^2
    // a	-54.3943883842204
    // b	-4.53956454545558
    // c	-0.0437470770400814
    public static double kRightACoeff = -54.3943883842204;
    public static double kRightBCoeff = -4.53956454545558;
    public static double kRightCCoeff = -0.0437470770400814;

    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable(tableName);

    @Override
    public void readInputs(double pNow) {
        boolean targetValid = mTable.getEntry("tv").getDouble(0.0) > 0.0;
        Robot.DATA.limelight.set(ELimelightData.TV, targetValid ? 1.0d : null);

        if(targetValid) {

//            if(mVisionTarget != null) {
//
//                Robot.DATA.limelight.set(ELimelightData.targetOrdinal, (double)mVisionTarget.ordinal());
//                Robot.DATA.limelight.set(ELimelightData.calcDistToTarget, calcTargetDistance(mVisionTarget));
//                Robot.DATA.limelight.set(calcAngleToTarget, calcTargetApproachAngle());
//                Optional<Translation2d> p = calcTargetLocation(mVisionTarget);
//                if(p.isPresent()) {
//                    Robot.DATA.limelight.set(ELimelightData.calcTargetX, p.get().getX());
//                    Robot.DATA.limelight.set(ELimelightData.calcTargetY, p.get().getY());
//                }
//            }
        }

    }

    @Override
    public void setOutputs(double pNow) {

    }

    @Override
    public Codex<Double, ELimelightData> getTargetingData() {
        return null;
    }

    @Override
    public double getCameraHeightIn() {
        return 0;
    }

    @Override
    public double getCameraAngleDeg() {
        return 0;
    }

    @Override
    public double getCameraToBumperIn() {
        return 0;
    }

    @Override
    public double getLeftCoeffA() {
        return 0;
    }

    @Override
    public double getLeftCoeffB() {
        return 0;
    }

    @Override
    public double getLeftCoeffC() {
        return 0;
    }

    @Override
    public double getRightCoeffA() {
        return 0;
    }

    @Override
    public double getRightCoeffB() {
        return 0;
    }

    @Override
    public double getRightCoeffC() {
        return 0;
    }
}
