package us.ilite.robot.controller;

import com.team319.trajectory.Path;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.swing.interop.SwingInterOpUtils;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.auto.paths.*;
import us.ilite.robot.modules.EDriveState;

import java.util.Map;

public class AutonCalibration extends BaseAutonController {

    public enum DisplayPaths {
        NONE(null),
        LOOP(new Loop()),
        OUR_TRENCH(new OurTrench()),
        SQUIGGLE(new Squiggle()),
        T_90DEG_12FT(new T_90DEG_12FT()),
        T_180DEG_24FT(new T_180DEG_24FT()),
        T_LINE_10_FT(new T_LINE_10_FT()),
        T_LINE_27_FT(new T_LINE_27_FT()),
        T_LINE_F10FT_L90DEG_F5FT_R5FT(new T_LINE_F10FT_L90DEG_F5FT_R5FT()),
        WONKY(new Wonky()),
        YOINK(new Yoink());

        public Path path;

        DisplayPaths(Path path) {
            this.path = path;
        }
    }

    private Map<String, Path> mPaths = BobUtils.getAvailablePaths();
    private ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Auton Config");
    private NetworkTableEntry mPathNumber = mAutonConfiguration.add("Path Number", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 9, "block increment", 1))
            .getEntry();
    private NetworkTableEntry mPathDelay = mAutonConfiguration.add("Path Delay Seconds", 0).getEntry();


    private final Distance mPathTotalDistance;
    private final double mMaxAllowedPathTime;
    private double mDelayCycleCount;

    public AutonCalibration() {
        int pathIndex = 0;
        mAutonConfiguration.addPersistent("Path Selection", "Select paths by clicking on the 'Path Number' slider dot and using arrow keys").withPosition(0, 1).withSize(4, 1);
        for (Map.Entry<String, Path> entry : mPaths.entrySet()) {
            mAutonConfiguration.addPersistent(entry.getKey(), pathIndex).withSize(1, 1).withPosition(pathIndex, 2);
            pathIndex++;
        }

        mActivePath = DisplayPaths.values()[mPathNumber.getNumber(0).intValue()].path;
        mDelayCycleCount = mPathDelay.getDouble(0.0) / .02;
        mPathTotalDistance = BobUtils.getPathTotalDistance(mActivePath);

        // Time to go through path plus any delay
        mMaxAllowedPathTime = BobUtils.getPathTotalTime(mActivePath) + 0.1 + (mDelayCycleCount * .02);

        e();
        System.out.println("==== RUNNING AUTONOMOUS PATH ====");
        System.out.println("Path: " + mActivePath.getClass().getSimpleName());
        System.out.println("Time (s): " + mMaxAllowedPathTime);
        System.out.println("Dist (ft): " + mPathTotalDistance);
        e();
    }

    @Override
    protected void updateImpl(double pNow) {
        if (mDelayCycleCount == 0) {
            if (mPathStartTime == 0) {
                mPathStartTime = pNow;
            }
//          Add a time check to prevent errors when things go wrong
            if(mActivePath != null && pNow - mPathStartTime <= mMaxAllowedPathTime) {
                int index = BobUtils.getIndexForCumulativeTime(mActivePath, pNow, mPathStartTime);
                if(index >= 0) {
                    db.drivetrain.set(EDriveData.STATE, EDriveState.PATH_FOLLOWING_BASIC);
                    db.drivetrain.set(EDriveData.L_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.LEFT_VELOCITY));
                    db.drivetrain.set(EDriveData.R_PATH_FT_s, mActivePath.getValue(index, Path.SegmentValue.RIGHT_VELOCITY));
                } else {
                    e();
                    System.out.println("==== SUCCESSFULLY END AUTONOMOUS PATH ====");
                    e();
                    mActivePath = null;
                }
            } else if(mActivePath != null && pNow - mPathStartTime > mMaxAllowedPathTime) {
                e();
                System.out.println("==== END AUTONOMOUS PATH DUE TO TIME OVERRUN ====");
                e();
            }
        } else {
            mDelayCycleCount--;
        }
    }

    private static final void e() {
        System.out.println("================================================");
    }
}
