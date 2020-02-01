package us.ilite.robot.controller;

import us.ilite.common.config.InputMap;
import us.ilite.common.types.EHangerSystemData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.FlywheelPrototype;
import us.ilite.robot.modules.HangerModule;

public class TestController extends AbstractController {

  //  private HangerModule mHanger;
    private HangerModule.EHangerState mHangerState;
    public TestController (){
   //     this.mHanger = pHanger;

    }

    public void update(double pNow) {
        updateHanger(pNow);

    }
    private void updateHanger(double pNow){
        if (Robot.DATA.operatorinput.isSet(InputMap.DRIVER.BEGIN_HANG)){
            Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER1 , 1.0);
            Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER2 , 1.0);

        }
        else if (Robot.DATA.operatorinput.isSet(InputMap.DRIVER.RELEASE_HANG)){
            Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER1 , 0.0);
            Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER2 , 0.0);

        }
        switch (mHangerState){
            case HANGING:
                Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER1 , 1.0);
                Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER2 , 1.0);
            case NOT_HANGING:
                Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER1 , 0.0);
                Robot.DATA.hanger.set(EHangerSystemData.DESIRED_HANGER_POWER2 , 0.0);
        }
    }
}
