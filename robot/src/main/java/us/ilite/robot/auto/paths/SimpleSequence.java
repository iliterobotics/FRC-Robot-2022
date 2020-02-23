package us.ilite.robot.auto.paths;

import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.IAutoCommand;
import us.ilite.robot.commands.ICommand;
import us.ilite.robot.controller.AbstractController;
import us.ilite.robot.modules.PowerCellModule;

import javax.swing.*;

import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;
import static us.ilite.robot.controller.AbstractController.kIntakeRollerPower_off;
import static us.ilite.robot.controller.AbstractController.kIntakeRollerPower_on;

public class SimpleSequence implements ISequence {
    private static int beamCounter = 0;
    private static boolean mLastEntryBeamBroken = false;
    private static boolean mEntryBeamBroken = false;
    private static IAutoCommand[] mSteps = {new IntakeCommand(2)};
    private static int mCurrentCommandIndex = 0;
    private double mCurrentDistance = 0;

    public static boolean checkBeams(){
        mEntryBeamBroken = Robot.DATA.powercell.get(EPowerCellData.ENTRY_BEAM) == 1.0;
        if (mEntryBeamBroken && !mLastEntryBeamBroken) {
            beamCounter++;
        }
        mLastEntryBeamBroken = mEntryBeamBroken;
        return beamCounter >= 2;
    }

    @Override
    public boolean isFinished() {
        return mCurrentCommandIndex == mSteps.length;
    }

    @Override
    public boolean finishedWithStep() {
        (mSteps[mCurrentCommandIndex]).updateDistance(mCurrentDistance);
        return mSteps[mCurrentCommandIndex].update(Robot.CLOCK.getCurrentTime());
    }

    @Override
    public boolean updateSequence(double pNow, double pDistance) {
        mCurrentDistance = pDistance;
        if (finishedWithStep()) {
            mCurrentCommandIndex++;
        }
        return isFinished();
    }

    static class IntakeCommand implements IAutoCommand {

        private double mActivationDistance = 0;
        private double mCurrentDistance = 0;

        @Override
        public void init(double pNow) {

        }

        public IntakeCommand ( double pActivateDistance ) {
            this.mActivationDistance = pActivateDistance;
        }

        @Override
        public boolean update(double pNow) {
            if (mCurrentDistance >= mActivationDistance ) {
                System.out.println("WORKING__________________________");
//                Robot.DATA.powercell.set(EPowerCellData.INTAKE_STATE , PowerCellModule.EArmState.STOW);
//                Robot.DATA.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , 0);
//                Robot.DATA.powercell.set(EPowerCellData.DESIRED_H_VELOCITY , 0.5);
//                Robot.DATA.powercell.set(EPowerCellData.DESIRED_V_VELOCITY , 0.5);

                double speed = Math.max(Robot.DATA.drivetrain.get(L_ACTUAL_VEL_FT_s), Robot.DATA.drivetrain.get(R_ACTUAL_VEL_FT_s));
                if(speed <= 1.0) {
                    speed = 0.3;
                }
//                Robot.DATA.powercell.set(INTAKE_STATE, PowerCellModule.EArm.OUT);
                Robot.DATA.powercell.set(SET_INTAKE_VEL_ft_s, kIntakeRollerPower_on);

                return checkBeams();
            }
            return false;
        }

        @Override
        public void updateDistance(double pCurrentDistance) {
            this.mCurrentDistance = pCurrentDistance;
        }

        @Override
        public void shutdown(double pNow) {

        }
    }
}
