package frc.team3128.subsystems.Led;

public enum LedStates {

    UNDEFINED(0, 0, 0, false),

    DISABLED(0, 0, 0, true), //RAINBOW -> disabled

    DEFAULT(0, 0, 0, true), //FLAME -> both neutrals, all primes + reef tracking

    CLIMB_PRIME(255, 50, 0, false), //ORANGE
    CLIMB(0, 255, 0, false), // GREEN

    AUTO_HOLD(255, 0, 246, false), // PINK

    INTAKE(0, 0, 255, false), // BLUE
    EJECT_OUTTAKE(153, 0, 255, false), // PURPLE

    PRIME(255, 50, 0, false), //ORANGE
    SCORE(0, 255, 0, false),
    
    REEF_1(0, 245, 255, false), // CYAN
    REEF_2(255, 255, 0, false), // YELLOW
    REEF_3(0, 255, 255, false), // CYAN
    REEF_4(255, 255, 0, false), // YELLOW
    REEF_5(0, 255, 255, false), // CYAN
    REEF_6(255, 255, 0, false), // YELLOW
    SOURCE(255, 0, 0, false); // RED

    public final int r;
    public final int b;
    public final int g;
    public final boolean animation;

    LedStates(int r, int g, int b, boolean animation) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.animation = animation;
    }

}