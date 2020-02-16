package us.ilite.robot.modules;

import com.flybotix.hfr.util.lang.EnumUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Data;
//import static us.ilite.common.types.ERawLimelightData.*;

import us.ilite.common.Field2020;
import us.ilite.common.IFieldComponent;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.ERawLimelightData;
import us.ilite.robot.Robot;

import static java.lang.Math.*;

public class Vision extends Module {
    private Limelight mLimelight;
    private final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    private Data mData;
    private IFieldComponent mTrackingType;

    private double lastXPosition = 0.0;
    private double lastYPosition = 0.0;
    private final double acceptableError = 0.1;
    private final double acceptableXError = 10.0;
    private final double acceptableYError = 10.0;
    private int selectedTarget = -1;

    private boolean isTracking = false;

    public Vision(Data pData, Limelight pLimelight) {
        mData = pData;
        mLimelight = pLimelight;
//        mTrackingType = mLimelight.getTracking();
    }
    @Override
    public void readInputs(double pNow) {

    }

    @Override
    public void setOutputs(double pNow) {
    }

    @Override
    public void shutdown(double pNow) {

    }

    public void sortTrackingData() {

    }
}