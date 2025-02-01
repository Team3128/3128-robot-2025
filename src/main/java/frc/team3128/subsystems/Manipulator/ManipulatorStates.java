package frc.team3128.subsystems.Manipulator;

import io.vavr.collection.List;

public enum ManipulatorStates {
    IDLE,
    NEUTRAL,
    IN(2),
    OUT(-6);

    private double volts;

    public static final List<ManipulatorStates> functionalStates = List.of(NEUTRAL, IN, OUT);

    private ManipulatorStates(double volts) {
        this.volts = volts;
    }

    private ManipulatorStates() {
        this.volts = 0;
    }

    public double getVolts() {
        return volts;
    }
}
