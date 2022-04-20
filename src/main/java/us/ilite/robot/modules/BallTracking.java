package us.ilite.robot.modules;

import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.*;
import us.ilite.common.types.EPixyData;

import java.util.ArrayList;
import java.util.Comparator;

public class BallTracking extends Module {
    Link link = new SPILink();
    Pixy2 pixy = Pixy2.createInstance(link);
    Block largestBlock = null;
    Block secondLargestBlock = null;

    public BallTracking() {
        pixy.init();
        System.out.println("pixy ccc " + pixy.getCCC());
        largestBlock = findLargestBLock();
        secondLargestBlock = findSecondLargestBLock();
    }

    public void readInputs() {
        db.pixydata.set(EPixyData.SIGNATURE, largestBlock.getSignature());

        db.pixydata.set(EPixyData.LARGEST_XCoorinate, largestBlock.getX());
        db.pixydata.set(EPixyData.LARGEST_YCoordniate, largestBlock.getY());
        db.pixydata.set(EPixyData.LARGEST_WIDTH, largestBlock.getWidth());
        db.pixydata.set(EPixyData.LARGEST_HEIGHT, largestBlock.getHeight());
        db.pixydata.set(EPixyData.LARGEST_ANGLE_FROM_CAMERA, largestBlock.getAngle());

        db.pixydata.set(EPixyData.SECOND_LARGEST_XCoorinate, secondLargestBlock.getX());
        db.pixydata.set(EPixyData.SECOND_LARGEST_YCoordniate, secondLargestBlock.getY());
        db.pixydata.set(EPixyData.SECOND_LARGEST_WIDTH, secondLargestBlock.getWidth());
        db.pixydata.set(EPixyData.SECOND_LARGEST_HEIGHT, secondLargestBlock.getHeight());
        db.pixydata.set(EPixyData.SECOND_LARGEST_ANGLE_FROM_CAMERA, secondLargestBlock.getAngle());
    }


    public Block findLargestBLock() {
        pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 50);
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        if (blocks.size() > 0) {
            db.pixydata.set(EPixyData.TARGET_VALID, 1);
            return blocks.get(blocks.size() - 1);
        } else {
            db.pixydata.set(EPixyData.TARGET_VALID, 0);
            return null;
        }
    }

    public Block findSecondLargestBLock() {
        pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 50);
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        if (blocks.size() > 1) {
            db.pixydata.set(EPixyData.TARGET_VALID, 1);
            return blocks.get(blocks.size() - 2);
        } else {
            db.pixydata.set(EPixyData.TARGET_VALID, 0);
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


