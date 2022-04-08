package us.ilite.robot.modules;

import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.*;
import us.ilite.common.types.ELargestPixyData;
import us.ilite.common.types.ESecondLargestPixyData;

import java.util.ArrayList;
import java.util.Comparator;

public class BallTracking extends Module {
    SPILink link = new SPILink();
    Pixy2 pixy = Pixy2.createInstance(link);
    Block largestBlock = findLargestBLock();
    Block secondLargestBlock = findSecondLargestBLock();

    public void readInputs() {
        db.largestpixydata.set(ELargestPixyData.XCoorinate, largestBlock.getX());
        db.largestpixydata.set(ELargestPixyData.YCoordniate, largestBlock.getY());
        db.largestpixydata.set(ELargestPixyData.WIDTH, largestBlock.getWidth());
        db.largestpixydata.set(ELargestPixyData.HEIGHT, largestBlock.getHeight());
        db.largestpixydata.set(ELargestPixyData.SIGNATURE, largestBlock.getSignature());
        db.largestpixydata.set(ELargestPixyData.ANGLE_FROM_CAMERA, largestBlock.getAngle());

        db.secondlargestpixydata.set(ESecondLargestPixyData.XCoorinate, secondLargestBlock.getX());
        db.secondlargestpixydata.set(ESecondLargestPixyData.YCoordniate, secondLargestBlock.getY());
        db.secondlargestpixydata.set(ESecondLargestPixyData.WIDTH, secondLargestBlock.getWidth());
        db.secondlargestpixydata.set(ESecondLargestPixyData.HEIGHT, secondLargestBlock.getHeight());
        db.secondlargestpixydata.set(ESecondLargestPixyData.SIGNATURE, secondLargestBlock.getSignature());
        db.secondlargestpixydata.set(ESecondLargestPixyData.ANGLE_FROM_CAMERA, secondLargestBlock.getAngle());
    }


    public Block findLargestBLock() {
//        int numBlocks = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        Block largestBlock = blocks.get(blocks.size() - 1);
        return largestBlock;
    }

    public Block findSecondLargestBLock() {
//        int numBlocks = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        final ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        blocks.sort(new Sorter());
        Block largestBlock = blocks.get(blocks.size() - 2);
        return largestBlock;
    }

    private final class Sorter implements Comparator<Block> {

        @Override
        public int compare(Block block, Block t1) {
            return 0;
        }
    }
}


