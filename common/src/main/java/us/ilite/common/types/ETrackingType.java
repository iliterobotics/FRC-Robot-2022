package us.ilite.common.types;

public enum ETrackingType {

    NONE(0, 0, true),
    TARGET(1, 0, true),
    TARGET_ZOOM( 2, 0 , false),
    BALL(3, 0, true),
    BALL_DUAL(5, 0, false),
    BALL_TRI(6, 0, false);

    private final int kPipelineNum;
    private final boolean kLedOn;
    private final int kTurnScalar;

    ETrackingType(int pPipelineNum, int pTurnScalar, boolean pLedOn) {
        kPipelineNum = pPipelineNum;
        kTurnScalar = pTurnScalar;
        kLedOn = pLedOn;
    }

    public int getPipeline() {
        return kPipelineNum;
    }

    public int getTurnScalar() {
        return kTurnScalar;
    }

    public boolean getLedOn() {
        return kLedOn;
    }

}