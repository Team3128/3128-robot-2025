package frc.team3128.subsystems.Elevator;
import io.vavr.collection.List;

public enum ElevatorStates {
    IDLE,
    NEUTRAL(0),
    SOURCE(0),
    L1(0.3),
    L2(0.475),
    L3(0.885),
    L4(1.55);

    private double setpoint;

    public static final List<ElevatorStates> functionalStates = List.of(NEUTRAL, L1, L2, L3, L4);

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
