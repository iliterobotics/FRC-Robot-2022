package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMusicModuleData;
import us.ilite.robot.Enums;


public class MusicModule extends Module {
    private Orchestra mTalonOrchestra;
    private String kSong;
    private TalonFX mIntakeRollers;
    private TalonFX mFeeder;
    private TalonFX mDrivetrainMasterLeft;
    private TalonFX mDrivetrainLeftFollower;
    private TalonFX mDrivetrainMasterRight;
    private TalonFX mDrivetrainRightFollower;

    public MusicModule() {
        kSong = "placeholder.chrp";
        mIntakeRollers = new TalonFX(Settings.HW.CAN.kINRoller);
        mFeeder = new TalonFX(Settings.HW.CAN.kINFeeder);
        mDrivetrainMasterLeft = new TalonFX(Settings.HW.CAN.kDTML1);
        mDrivetrainLeftFollower = new TalonFX(Settings.HW.CAN.kDTL3);
        mDrivetrainMasterRight = new TalonFX(Settings.HW.CAN.kDTMR2);
        mDrivetrainRightFollower = new TalonFX(Settings.HW.CAN.kDTR4);
        mTalonOrchestra = new Orchestra();
        mTalonOrchestra.loadMusic(kSong);
        mTalonOrchestra.addInstrument(mIntakeRollers);
        mTalonOrchestra.addInstrument(mFeeder);
        mTalonOrchestra.addInstrument(mDrivetrainMasterLeft);
        mTalonOrchestra.addInstrument(mDrivetrainMasterRight);
        mTalonOrchestra.addInstrument(mDrivetrainLeftFollower);
        mTalonOrchestra.addInstrument(mDrivetrainRightFollower);
    }

    public void readInputs() {

    }

    @Override
    public void setOutputs() {
        Enums.EMusicState mode = db.music.get(EMusicModuleData.STATE, Enums.EMusicState.class);
        if (mode == null) {
            return;
        }
        switch (mode) {
            case TARGET_LOCK:
                //TODO figure out different songs
                kSong = "placeholder.chrp";
                mTalonOrchestra.loadMusic(kSong);
                mTalonOrchestra.play();
                break;
            case HANGING:
                kSong = "placeholder.chrp";
                mTalonOrchestra.loadMusic(kSong);
                mTalonOrchestra.play();
                break;
            case CURRENT_LIMITING:
                kSong = "placeholder.chrp";
                mTalonOrchestra.loadMusic(kSong);
                mTalonOrchestra.play();
                break;
        }
    }
}