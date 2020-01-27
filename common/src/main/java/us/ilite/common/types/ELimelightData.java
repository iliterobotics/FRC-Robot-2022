
package us.ilite.common.types;


import com.flybotix.hfr.codex.CodexOf;

public enum ELimelightData implements CodexOf<Double> {
    // TODO - document what these values represent
    tv,
    tx,
    ty,
    ta,
    ts,
    tl,
    tshort,
    tlong,
    thoriz,
    tvert,
    
    targetOrdinal,
    calcDistToTarget,
    calcAngleToTarget,
    calcTargetX,
    calcTargetY,

    TRACKING_TYPE;
}