package frc.team3128.subsystems.Led;

import io.vavr.collection.List;

import static frc.team3128.Constants.LedConstants.*;

public enum LedStates {
    OFF(0,0,0,false),
    ERROR(255, 0, 0, false),
    PIECE(0, 255, 0, false),
    CONFIGURED(0,255,0,false),
    BLUE(48, 122, 171, false),
    RED(171, 48, 97, false),
    PURPLE(255, 0, 255, false),
    GREEN(0, 255, 0, false),
    ORANGE(255, 50, 0, false),


    UNDEFINED(0, 0, 0, false), //RAINBOW
    NEUTRAL(0,0,0,true), //FLAME
    REEF_PRIME(255, 0, 0, false), //PROXIMITY
    CLIMB_PRIME(0,255,0, false),
    CLIMB(0,0,255, false),
    RPL1(0,255,0,true),
    RPL2(0,255,0,true),
    RPL3(0,255,0,true),
    RPL4(0,255,0,true),
    RSL1(255,0,0,false),
    RSL2(255,0,0,false),
    RSL3(255,0,0,false),
    RSL4(255,0,0,false),
    INTAKE(255,0,0,true),
    EJECT_OUTTAKE(255,0,0,true),
    PROCESSOR_PRIME(0,255,0,true),
    PROCESSOR_OUTTAKE(0,255,0,true),
    REEF_SCORE(0, 255, 0, false);
    

    public final int r;
    public final int b;
    public final int g;
    public final boolean animation;

    public static List<LedStates> functionalStates = List.of(UNDEFINED, NEUTRAL, REEF_PRIME, REEF_SCORE, CLIMB_PRIME);

    LedStates(int r, int g, int b, boolean animation) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.animation = animation;
    }

}