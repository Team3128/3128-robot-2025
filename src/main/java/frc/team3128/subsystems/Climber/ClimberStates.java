package frc.team3128.subsystems.Climber;

public enum ClimberStates {
    UNDEFINED,
    NEUTRAL,
    CLIMB_PRIME(90.0,false,false),
    CLIMB(0.0,true,true);

    private double angle;
    private boolean hasClaw;
    private boolean hasWinch;

    private ClimberStates(double angle, boolean hasClaw, boolean hasWinch) {
        this.angle = angle;
        this.hasClaw = hasClaw;
        this.hasWinch = hasWinch;
    }

    private ClimberStates(double angle) {
        this(angle, false, false);
    }

    private ClimberStates() {
        this(0);
    }

    public double getAngle() {
        return angle;
    }

    public boolean getHasClaw() {
        return hasClaw;
    }

    public boolean getHasWinch() {
        return hasWinch;
    }
}
