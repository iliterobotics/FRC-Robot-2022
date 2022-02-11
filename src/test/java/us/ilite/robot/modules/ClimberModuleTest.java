package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.flybotix.hfr.codex.RobotCodex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import us.ilite.common.types.EHangerModuleData;

import java.io.IOException;

import static org.mockito.Mockito.*;

public class ClimberModuleTest {

    private ClimberModule mClimber;
    private TalonFX mMockedTalonFX;
    private RobotCodex<EHangerModuleData> mHangerModule;
    private RelativeEncoder mMockedRelativeEncoder;

    @Before
    public void setup() {
        mMockedTalonFX = mock(TalonFX.class);
        mHangerModule = mock(RobotCodex.class);
        mMockedRelativeEncoder = mock(RelativeEncoder.class);
        mClimber = new ClimberModule(mMockedTalonFX, mHangerModule, mMockedRelativeEncoder);
    }

    @Test
    public void testReadInputs() {
        when(mMockedRelativeEncoder.getVelocity()).thenReturn(50d);

        mClimber.readInputs();

        verify(mHangerModule, times(1)).set(EHangerModuleData.L_VEL_rpm, 50d);
    }
    @Test
    public void testHandleInputs_IOException() {
        when(mMockedRelativeEncoder.getVelocity()).thenThrow(RuntimeException.class);

        mClimber.readInputs();

        verify(mHangerModule, never()).set(EHangerModuleData.L_VEL_rpm, 50d);
    }
    @Test
    public void testSetOutputs() {

    }
}
