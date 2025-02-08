package frc.team3128.subsystems.Climber;

public enum ClimberStates {
    UNDEFINED,
    NEUTRAL,
    // CLIMB_PRIME(90.0,false,false),
    // CLIMB(0.0,true,true);
    CLIMB_PRIME(90.0, true),
    CLIMB(0.0, false);

    private double angle;
    private boolean hasWinch;
    private boolean hasRoller;

    private ClimberStates(double angle, boolean hasRoller) {
        this.angle = angle;
        this.hasRoller = hasRoller;
    }

    private ClimberStates(double angle) {
        this(angle, false);
    }

    private ClimberStates() {
        this(0);
    }

    public double getAngle() {
        return angle;
    }

    public boolean getHasWinch() {
        return hasWinch;
    }

    public boolean getHasRoller() {
        return hasRoller;
    }
}
