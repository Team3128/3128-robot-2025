package frc.team3128.subsystems.Elevator;

public enum ElevatorStates {
    IDLE,
    NEUTRAL(0),
    SOURCE(0),
    L1(0),
    L2(0),
    L3(0),
    L4(0);

    private double setpoint;

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
