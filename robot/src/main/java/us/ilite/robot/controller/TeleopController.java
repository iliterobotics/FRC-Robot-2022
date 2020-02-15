package us.ilite.robot.controller;

public class TeleopController extends BaseManualController {

    private static TeleopController INSTANCE;

    public static TeleopController getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {

    }

    @Override
    protected void updateImpl(double pNow) {
        super.updateDrivetrain(pNow);
    }
}
