package frc.team3128.subsystems.Climber;

public enum ClimberStates {
    UNDEFINED,
    NEUTRAL,
    CLIMB_PRIME(8000.0, 0.5, 0.5),
    CLIMB(0.0, -0.5, 0);

    private double angle;
    private double winchPower;
    private double rollerPower;

    private ClimberStates(double angle, double winchPower, double rollerPower) {
        this.angle = angle;
        this.winchPower = winchPower;
        this.rollerPower = rollerPower;
    }

    private ClimberStates() {
        this(0, 0, 0);
    }

    public double getAngle() {
        return this.angle;
    }

    public double getWinchPower() {
        return this.winchPower;
    }

    public double getRollerPower() {
        return this.rollerPower;
    }
}
