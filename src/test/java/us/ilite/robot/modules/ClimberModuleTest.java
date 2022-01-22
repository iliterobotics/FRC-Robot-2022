package us.ilite.robot.modules;

import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import us.ilite.common.types.EHangerModuleData;

import static org.mockito.Mockito.*;

public class ClimberModuleTest {

    private ClimberModule mClimber;
    private CANSparkMax mMockedCANSparkMax;
    private RobotCodex<EHangerModuleData> mHangerModule;
    private RelativeEncoder mMockedRelativeEncoder;

    @Before
    public void setup() {
        mMockedCANSparkMax = mock(CANSparkMax.class);
        mHangerModule = mock(RobotCodex.class);
        mMockedRelativeEncoder = mock(RelativeEncoder.class);
        mClimber = new ClimberModule(mMockedCANSparkMax, mHangerModule, mMockedRelativeEncoder);
    }

    @Test
    public void testReadInputs() {

    }
    @Test
    public void testSetOutputs() {

    }
}
