package frc.team3128.subsystems.Manipulator;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.COAST;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum ManipulatorStates {
    UNDEFINED,
    NEUTRAL(0),
    IN(4),
    OUT(-6);

    private double power;
    private Neutral neutral;

    public static final List<ManipulatorStates> functionalStates = List.of(NEUTRAL, IN, OUT);

    private ManipulatorStates(double power) {
        this.power = power;
        this.neutral = Neutral.BRAKE;
    }

    private ManipulatorStates() {
        this.power = 0;
        this.neutral = COAST;
    }

    public double getPower() {
        return this.power;
    }

    public Neutral getNeutral() {
        return this.neutral;
    }
}
