package frc.team3128.subsystems.Elevator;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum ElevatorStates {
    UNDEFINED,
    NEUTRAL(0, 0),
    LOW_L1(0.13),
    HIGH_L1(0.29),
    L2(0.47, 0.51),
    L3(0.865, 0.9),
    L4(1.531, 1.531),
    A1(0.1),
    A2(0.5),
    AB(1.501),
    TELE_HOLD(0.845),
    AUTO_HOLD(1.531);

    private double setpointRam;
    private double setpointRamless;

    public static final List<ElevatorStates> functionalStates = List.of(NEUTRAL, LOW_L1, HIGH_L1, L2, L3, L4, A1, A2, AB, TELE_HOLD, AUTO_HOLD);

    private ElevatorStates(double setpointRam, double setpointRamless) {
        this.setpointRam = setpointRam;
        this.setpointRamless = setpointRamless;
    }

    private ElevatorStates(double setpoint) {
        this(setpoint, setpoint);
    }

    private ElevatorStates() {
        this(0, 0);
    }

    public double getSetpointRam() {
        return this.setpointRam;
    }

    public double getSetpointRamless() {
        return this.setpointRamless;
    }
}