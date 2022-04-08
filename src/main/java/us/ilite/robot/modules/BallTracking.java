package us.ilite.robot.modules;

import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.*;
import us.ilite.common.types.EPixyData;

import java.util.ArrayList;
import java.util.Comparator;

public class BallTracking extends Module {
    private Pixy2 pixy;
    private int state = -1 ;
    boolean isCamera = false;

    public BallTracking() {
//        pixy.init();
        pixy = Pixy2.createInstance(Pixy2.LinkType.SPI);
    }

    public void readInputs() {

        if(!isCamera) {
            state = pixy.init(0);
            isCamera = state >=0;
            System.out.println("state: " + state);
        }
        if (!isCamera) {
            return;
        }

        Block largestBlock = findLargestBLock();
        Block secondLargestBlock = findSecondLargestBLock();

        if (largestBlock != null) {
            db.pixydata.set(EPixyData.LARGEST_XCoorinate, largestBlock.getX());
            db.pixydata.set(EPixyData.LARGEST_YCoordniate, largestBlock.getY());
            db.pixydata.set(EPixyData.LARGEST_WIDTH, largestBlock.getWidth());
            db.pixydata.set(EPixyData.LARGEST_HEIGHT, largestBlock.getHeight());
            db.pixydata.set(EPixyData.LARGEST_SIGNATURE, largestBlock.getSignature());
            db.pixydata.set(EPixyData.LARGEST_ANGLE_FROM_CAMERA, largestBlock.getAngle());
        } else {
            System.out.println("largest block is null");
        }

        if (secondLargestBlock != null) {
            db.pixydata.set(EPixyData.SECOND_LARGEST_XCoorinate, secondLargestBlock.getX());
            db.pixydata.set(EPixyData.SECOND_LARGEST_YCoordniate, secondLargestBlock.getY());
            db.pixydata.set(EPixyData.SECOND_LARGEST_WIDTH, secondLargestBlock.getWidth());
            db.pixydata.set(EPixyData.SECOND_LARGEST_HEIGHT, secondLargestBlock.getHeight());
            db.pixydata.set(EPixyData.SECOND_LARGEST_SIGNATURE, secondLargestBlock.getSignature());
            db.pixydata.set(EPixyData.SECOND_LARGEST_ANGLE_FROM_CAMERA, secondLargestBlock.getAngle());
        } else {
            System.out.println("second largest block is null");
        }
    }

    public void setOutputs() {
        pixy.setCameraBrightness(255);
    }


    public Block findLargestBLock() {
        Pixy2CCC pixyccc = pixy.getCCC();
        pixyccc.getBlocks(true, Pixy2CCC.CCC_SIG2, 25);
//        System.out.println("found " + blockCount + " blocks");
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        if (blocks.size() > 0) {
            return blocks.get(blocks.size() - 1);
        } else {
            return null;
        }
    }

    public Block findSecondLargestBLock() {
        pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG2, 25);
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        if (blocks.size() > 1) {
            return blocks.get(blocks.size() - 2);
        } else {
            return null;
        }
    }

    private final class Sorter implements Comparator<Block> {

        @Override
        public int compare(Block block, Block t1) {
            return 0;
        }
    }
}


