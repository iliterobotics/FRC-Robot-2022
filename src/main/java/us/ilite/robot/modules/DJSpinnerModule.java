package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EColorData;
import us.ilite.robot.hardware.TalonSRXFactory;
import static us.ilite.robot.Enums.*;

public class DJSpinnerModule extends Module {

    public static final double TARGET_ROTATION_COUNT = 32.0;
    private ILog mLog = Logger.createLog( this.getClass());

    private VictorSPX mVictor;
    private ColorSensorV3 mColorSensorV3;
    private final ColorMatch mColorMatcher = new ColorMatch();
    private EColorMatch eDesiredColorState;
    private EColorMatch eCurrentColorState;
    private EColorMatch eLastColorState;
    private double mSolidStateCounter;
    private double mColorChangeCounter;
    private EColorWheelState eColorWheelState;


    public DJSpinnerModule() {

        I2C.Port i2cPort = I2C.Port.kOnboard;
        mColorSensorV3 = new ColorSensorV3(i2cPort);

        mSolidStateCounter = 0;
        eColorWheelState = EColorWheelState.OFF;
        eCurrentColorState = EColorMatch.NONE;
        eLastColorState = EColorMatch.NONE;
        eDesiredColorState = EColorMatch.NONE;
        mColorChangeCounter = 0;
        mVictor = TalonSRXFactory.createDefaultVictor(Settings.HW.CAN.kDJSpinnerVictorID);
        mVictor.setNeutralMode(NeutralMode.Brake);


        for(EColorMatch cm : EColorMatch.values()) {
            if(cm != EColorMatch.NONE) {
                mColorMatcher.addColorMatch(cm.color);
            }
        }
    }


    @Override
    public void readInputs() {
        Color c = mColorSensorV3.getColor();
        db.color.set(EColorData.MEASURED_BLUE, c.blue);
        db.color.set(EColorData.MEASURED_GREEN, c.green);
        db.color.set(EColorData.MEASURED_RED, c.red);
        ColorMatchResult match = mColorMatcher.matchClosestColor(c);
        db.color.set(EColorData.SENSED_COLOR, EColorMatch.from(match));
        eCurrentColorState = EColorMatch.from(match);
        db.color.set(EColorData.WHEEL_ROTATION_COUNT, mColorChangeCounter);
        db.color.set(EColorData.CURRENT_MOTOR_POWER , mVictor.getMotorOutputPercent());
    }

    @Override
    public void setOutputs() {
        mVictor.set(ControlMode.PercentOutput, db.color.get(EColorData.DESIRED_MOTOR_POWER));

        if( db.color.get(EColorData.COLOR_WHEEL_MOTOR_STATE) == EColorWheelState.ROTATION.ordinal() ) {
            if ( eLastColorState.nextColor() == EColorMatch.NONE ) {
                eLastColorState = eCurrentColorState;
            }
            else if (eLastColorState.nextColor() == eCurrentColorState) {
                mColorChangeCounter++;
            }
            else if ( !eCurrentColorState.equals(eLastColorState) ) {
                eCurrentColorState = eLastColorState;
            }
            eLastColorState = eCurrentColorState;
            db.color.set( EColorData.UNDERSTOOD_COLOR, eCurrentColorState.ordinal() );
        }
    }


}
