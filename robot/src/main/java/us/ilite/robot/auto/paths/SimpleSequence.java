package us.ilite.robot.auto.paths;

import us.ilite.common.types.EPowerCellData;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.IAutoCommand;
import us.ilite.robot.commands.ICommand;

import javax.swing.*;

public class SimpleSequence implements ISequence {
    private static int beamCounter = 0;
    private static boolean mLastEntryBeamBroken = false;
    private static boolean mEntryBeamBroken = false;
    private static IAutoCommand[] mSteps = {new IntakeCommand(0.0)};
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
            if ( mCurrentDistance == mActivationDistance ) {
                Robot.DATA.powercell.set(EPowerCellData.CURRENT_ARM_ANGLE , 0);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_H_VELOCITY , 0.5);
                Robot.DATA.powercell.set(EPowerCellData.DESIRED_V_VELOCITY , 0.5);
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
