package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EColorData;
import us.ilite.robot.hardware.TalonSRXFactory;
import static us.ilite.robot.Enums.*;

public class DJSpinnerModule extends Module {

    public static final double TARGET_ROTATION_COUNT = 32.0;
    private ILog mLog = Logger.createLog( this.getClass());



    public enum EIsFinished {
        YES,
        NO
    }

    private VictorSPX mVictor;
    private ColorSensorV3 mColorSensorV3;
    private final ColorMatch mColorMatcher = new ColorMatch();
    private EColorMatch eDesiredColorState;
    private EColorMatch eCurrentColorState;
    private EColorMatch eLastColorState;
    private double mSolidStateCounter;
    private double mColorChangeCounter = 0;
    private EColorWheelState eColorWheelState;
    private boolean mIsDone;
    private boolean mMightBeDone;
    private int mRedTracker;
    private int mBlueTracker;
    private int mGreenTracker;
    private int mYellowTracker;
    private int total;


    public DJSpinnerModule() {

        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);

        mSolidStateCounter = 0;
        eColorWheelState = EColorWheelState.OFF;
        eCurrentColorState = EColorMatch.NONE;
        eLastColorState = EColorMatch.NONE;
        eDesiredColorState = EColorMatch.NONE;
        mIsDone = false;
        mMightBeDone = false;
        mVictor = TalonSRXFactory.createDefaultVictor(Settings.Hardware.CAN.kDJSpinnerVictorID);
        mVictor.setNeutralMode(NeutralMode.Brake);


        for(EColorMatch cm : EColorMatch.values()) {
            if(cm != EColorMatch.NONE) {
                mColorMatcher.addColorMatch(cm.color);
            }
        }
    }


    @Override
    public void readInputs(double pNow) {
        Color c = mColorSensorV3.getColor();
        db.color.set(EColorData.MEAURED_BLUE, c.blue);
        db.color.set(EColorData.MEAURED_GREEN, c.green);
        db.color.set(EColorData.MEAURED_RED, c.red);
        ColorMatchResult match = mColorMatcher.matchClosestColor(c);
        db.color.set(EColorData.SENSED_COLOR, EColorMatch.from(match));
        db.color.set(EColorData.WHEEL_ROTATION_COUNT, total);
        db.color.set(EColorData.CURRENT_MOTOR_POWER , mVictor.getMotorOutputPercent());
        SmartDashboard.putString("Detected Color: ", getEnumOfOrdinal( db.color.get( EColorData.SENSED_COLOR ) ).name() );

        if(db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE, EColorWheelState.class) == EColorWheelState.ROTATION) {
            SmartDashboard.putNumber("Number of color changes" , db.color.get(EColorData.WHEEL_ROTATION_COUNT));
            if(eLastColorState.nextColor() == eCurrentColorState) {
                incrementStateIterator( eCurrentColorState );
                mColorChangeCounter++;
            }
            eLastColorState = eCurrentColorState;
        }
    }

    public void incrementStateIterator( EColorMatch colorMatch ) {
        if ( colorMatch.ordinal() == EColorMatch.RED.ordinal() ) {
            mRedTracker++;
        }
        if ( colorMatch.ordinal() == EColorMatch.GREEN.ordinal() ) {
            mGreenTracker++;
        }if ( colorMatch.ordinal() == EColorMatch.BLUE.ordinal() ) {
            mBlueTracker++;
        }if ( colorMatch.ordinal() == EColorMatch.YELLOW.ordinal() ) {
            mYellowTracker++;
        }
        total = mBlueTracker + mYellowTracker + mGreenTracker + mRedTracker;
        SmartDashboard.putNumber("TOTAL COLORS" , total);
        if (total >= 32 ){
            mVictor.set(ControlMode.PercentOutput , 0.0);
        }


    }

    public EColorMatch getEnumOfOrdinal( double d ) {
//        if ( d == EColorMatch.RED.ordinal() ) {
//            return EColorMatch.RED;
//        }
//        else if ( d == EColorMatch.BLUE.ordinal() ) {
//            return EColorMatch.BLUE;
//        }
//        else if ( d == EColorMatch.GREEN.ordinal() ) {
//            return EColorMatch.GREEN;
//        }
//        else if ( d == EColorMatch.YELLOW.ordinal() ) {
//            return EColorMatch.YELLOW;
//        }
//        else {
//            return EColorMatch.NONE;
//        }
        return EColorMatch.values()[(int) d];
    }

    @Override
    public void setOutputs(double pNow) {
        mVictor.set(ControlMode.PercentOutput, db.color.get(EColorData.DESIRED_MOTOR_POWER));

    }


}
