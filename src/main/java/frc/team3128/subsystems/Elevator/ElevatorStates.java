package frc.team3128.subsystems.Elevator;

public enum ElevatorStates {
    IDLE,
    NEUTRAL(0),
    SOURCE(0.1),
    L1(0.2),
    L2(0.3),
    L3(0.4),
    L4(0.5);

    private double setpoint;

    public static ElevatorStates[] functionalStates = {NEUTRAL, SOURCE, L1, L2, L3, L4};

    private ElevatorStates(double setpoint) {
        this.setpoint = setpoint;
    }

    private ElevatorStates() {
        this.setpoint = 0;
    }

    public double getSetpoint() {
        return setpoint;
    }
}
