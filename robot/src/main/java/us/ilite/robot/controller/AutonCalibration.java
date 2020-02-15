package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import jdk.swing.interop.SwingInterOpUtils;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.auto.paths.*;
import us.ilite.robot.modules.EDriveState;

import java.util.Map;

public class AutonCalibration extends BaseAutonController {

    public enum PATHS {
        Loop(new Loop()),
        OurTrench(new OurTrench()),
        Squiggle(new Squiggle()),
        T_90DEG_12FT(new T_90DEG_12FT()),
        T_180DEG_24FT(new T_180DEG_24FT()),
        T_LINE_10_FT(new T_LINE_10_FT()),
        T_LINE_27_FT(new T_LINE_27_FT()),
        T_LINE_F10FT_L90DEG_F5FT_R5FT(new T_LINE_F10FT_L90DEG_F5FT_R5FT()),
        Wonky(new Wonky()),
        Yoink(new Yoink());

        Path path;

        PATHS(Path path) {
            this.path = path;
        }
    }

    private ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");
    private NetworkTableEntry mPathNumber = mAutonConfiguration.add("Path Number", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    private final Distance mPathTotalDistance;
    private final double mMaxAllowedPathTime;

    public AutonCalibration() {
        mActivePath = PATHS.Loop.path;
        mPathTotalDistance = BobUtils.getPathTotalDistance(mActivePath);
        mMaxAllowedPathTime = BobUtils.getPathTotalTime(mActivePath) + 0.1;

        e();
        System.out.println("==== RUNNING AUTONOMOUS PATH ====");
        System.out.println("Path: " + mActivePath.getClass().getSimpleName());
        System.out.println("Time (s): " + mMaxAllowedPathTime);
        System.out.println("Dist (ft): " + mPathTotalDistance);
        e();
    }

    @Override
    protected void updateImpl(double pNow) {
        if(mPathStartTime == 0) {
            mPathStartTime = pNow;
        }
        int pathNumber = mPathNumber.getNumber(999).intValue();
        mPathNumber.setString(PATHS.values()[pathNumber].name());
        // Add a time check to prevent errors when things go wrong
//        if(mActivePath != null && pNow - mPathStartTime <= mMaxAllowedPathTime) {
//            int index = BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime);
//            if(index >= 0) {
//                db.drivetrain.set(EDriveData.STATE, EDriveState.PATH_FOLLOWING_BASIC);
//                db.drivetrain.set(EDriveData.L_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.LEFT_VELOCITY));
//                db.drivetrain.set(EDriveData.R_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.RIGHT_VELOCITY));
//            } else {
//                e();
//                System.out.println("==== SUCCESSFULLY END AUTONOMOUS PATH ====");
//                e();
//                mActivePath = null;
//            }
//        } else if(mActivePath != null && pNow - mPathStartTime > mMaxAllowedPathTime) {
//            e();
//            System.out.println("==== END AUTONOMOUS PATH DUE TO TIME OVERRUN ====");
//            e();
//        }
    }

    private static final void e() {
        System.out.println("================================================");
    }
}
