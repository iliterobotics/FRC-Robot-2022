package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import us.ilite.common.config.Settings;

import static us.ilite.robot.Enums.*;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EMatchMode;


public class TalonSoundModule extends Module {



    private Orchestra mTalonOrch;
    private String kSong;
    private TalonFX mTalonSoundOut;
    private TalonFX mTalonSoundOut2;

    public TalonSoundModule() {
        kSong = "placeholder.chrp";
        mTalonSoundOut = new TalonFX(13);
        mTalonSoundOut2 = new TalonFX(14);
        mTalonOrch = new Orchestra();
        mTalonOrch.loadMusic(kSong);
        mTalonOrch.addInstrument(mTalonSoundOut);
        mTalonOrch.addInstrument(mTalonSoundOut2);
    }

    public void readInputs() {

    }

    @Override
    public void setOutputs() {
        mTalonOrch.play();
    }
}