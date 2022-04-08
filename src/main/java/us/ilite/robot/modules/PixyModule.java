package us.ilite.robot.modules;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyModule extends Module {

    public static Pixy2 mPixy;

    public PixyModule(){
        mPixy = Pixy2.createInstance(new SPILink());
        mPixy.init();
        mPixy.setLamp((byte)1, (byte)1);
        mPixy.setLED(0, 0, 255);
    }
}
