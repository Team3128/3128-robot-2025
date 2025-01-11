package frc.team3128.subsystems.Climber;

public enum ClimberStates {
    IDLE,
    NEUTRAL,
    CLIMB_PRIME,
    CLIMB_LOCKED,
    CLIMB_WINCH;

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
