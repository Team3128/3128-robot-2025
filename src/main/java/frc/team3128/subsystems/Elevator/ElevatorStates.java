package frc.team3128.subsystems.Elevator;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum ElevatorStates {
    IDLE,
    NEUTRAL(0),
    L1(0.3),
    L2(0.475),
    L3(0.885),
    L4(1.55);

    private double setpoint;
    private Neutral neutral;

    public static final List<ElevatorStates> functionalStates = List.of(NEUTRAL, L1, L2, L3, L4);

    private ElevatorStates(double setpoint) {
        this.setpoint = setpoint;
        this.neutral = Neutral.BRAKE;
    }

    private ElevatorStates() {
        this.setpoint = 0;
        this.neutral = Neutral.COAST;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public Neutral getNeutral() {
        return this.neutral;
    }
}
