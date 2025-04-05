package frc.team3128.subsystems.Elevator;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum ElevatorStates {
    UNDEFINED,
    NEUTRAL(0),
    LOW_L1(0.13),
    HIGH_L1(0.29),
    L2(0.49),
    L3(0.9),
    L4(1.53),//1.501
    TELE_HOLD(0.65),
    AUTO_HOLD(1.501);

    private double setpoint;
    private Neutral neutral;

    public static final List<ElevatorStates> functionalStates = List.of(NEUTRAL, LOW_L1, HIGH_L1, L2, L3, L4, TELE_HOLD, AUTO_HOLD);

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
