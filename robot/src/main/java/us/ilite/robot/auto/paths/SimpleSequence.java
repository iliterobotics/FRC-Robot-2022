package us.ilite.robot.auto.paths;

import us.ilite.robot.Robot;
import us.ilite.robot.commands.IAutoCommand;
import static us.ilite.common.types.EPowerCellData.*;
import static us.ilite.common.types.drive.EDriveData.L_ACTUAL_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_VEL_FT_s;
import static us.ilite.robot.controller.AbstractController.kIntakeRollerPower_on;

public class SimpleSequence implements ISequence {
    private static int beamCounter = 0;
    private static boolean mLastSecondaryBeamBroken = false;
    private static boolean mSecondaryBeamBroken = false;
    private static IAutoCommand[] mSteps = {new IntakeCommand(2)};
    private static int mCurrentCommandIndex = 0;
    private double mCurrentDistance = 0;

    public static boolean checkBeams(){
        mSecondaryBeamBroken = Robot.DATA.powercell.get(EXIT_BEAM) == 1.0;
        if (mSecondaryBeamBroken&&!mLastSecondaryBeamBroken) {
            beamCounter++;
        }
        mLastSecondaryBeamBroken = mSecondaryBeamBroken;
        System.out.println(beamCounter);
        return beamCounter >= 2;
    }

    @Override
    public boolean isFinished() {
        return mCurrentCommandIndex >= mSteps.length;
    }

    @Override
    public boolean finishedWithStep() {
        if (!isFinished()) {
            (mSteps[mCurrentCommandIndex]).updateDistance(mCurrentDistance);
            return mSteps[mCurrentCommandIndex].update(Robot.CLOCK.getCurrentTime());
        }
        return false;
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
                double speed = Math.max(Robot.DATA.drivetrain.get(L_ACTUAL_VEL_FT_s), Robot.DATA.drivetrain.get(R_ACTUAL_VEL_FT_s));
                if(speed <= 1.0) {
                    speed = 0.3;
                }
//                Robot.DATA.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.OUT);
                Robot.DATA.powercell.set(INTAKE_VEL_ft_s, kIntakeRollerPower_on);
                Robot.DATA.powercell.set(SET_H_pct, 0.5);
                Robot.DATA.powercell.set(SET_V_pct ,0.5);

                return checkBeams();
            } else {
                Robot.DATA.powercell.set(SET_H_pct, 0.0);
                Robot.DATA.powercell.set(SET_V_pct, 0.0);
//                Robot.DATA.powercell.set(INTAKE_STATE, PowerCellModule.EArmState.STOW);
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
