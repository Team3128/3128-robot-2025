package frc.team3128.subsystems.Manipulator;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.COAST;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum ManipulatorStates {
    UNDEFINED,
    NEUTRAL(0),
    IN(4),
    OUT(-6),
    OUT_L1(-4);

    private double volts;
    private Neutral neutral;

    public static final List<ManipulatorStates> functionalStates = List.of(NEUTRAL, IN, OUT, OUT_L1);

    private ManipulatorStates(double volts) {
        this.volts = volts;
        this.neutral = Neutral.BRAKE;
    }

    private ManipulatorStates() {
        this.volts = 0;
        this.neutral = COAST;
    }

    public double getVolts() {
        return this.volts;
    }

    public Neutral getNeutral() {
        return this.neutral;
    }
}
