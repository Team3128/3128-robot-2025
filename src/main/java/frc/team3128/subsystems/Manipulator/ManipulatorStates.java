package frc.team3128.subsystems.Manipulator;

import io.vavr.collection.List;

public enum ManipulatorStates {
    IDLE,
    NEUTRAL,
    IN(0.3),
    OUT(-0.5);

    private double power;

    public static final List<ManipulatorStates> functionalStates = List.of(NEUTRAL, IN, OUT);

    private ManipulatorStates(double power) {
        this.power = power;
    }

    private ManipulatorStates() {
        this.power = 0;
    }

    public double getPower() {
        return power;
    }
}
