package us.ilite.common.types;

import com.flybotix.hfr.codex.CodexOf;

public enum EFeederData implements CodexOf<Double>{
    ENTRY_BEAM,
    EXIT_BEAM,
    STATE,
    RESET_BALLS,
    NUM_BALLS,
    SET_VELOCITY_ft_s,
    SET_FEEDER_pct,
    ACTUAL_FEEDER_pct,
    EXIT_BALL_VELOCITY_ft_s;
}
