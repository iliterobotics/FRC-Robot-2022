package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EFeederData implements CodexOf<Double>{
    ENTRY_BEAM,
    EXIT_BEAM,
    NUM_BALLS,
    STATE,

    SET_FEEDER_pct,
    FEEDER_pct,
    EXIT_BALL_VELOCITY_ft_s
}
