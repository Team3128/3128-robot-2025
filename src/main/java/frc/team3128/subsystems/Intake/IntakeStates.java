package frc.team3128.subsystems.Intake;

import io.vavr.collection.List;

public enum IntakeStates {
    IDLE,
    NEUTRAL(90),
    INTAKE(0, 1),
    EJECT_OUTTAKE(90, -1),
    PROCESSOR_PRIME(999),
    PROCESSOR_OUTTAKE(999, 999),
    CLIMB_PRIME(999),
    CLIMB_LOCK(999),
    CLIMB_WINCH(666);

    public static final List<IntakeStates> defaultIntakeStates = List.of(INTAKE, EJECT_OUTTAKE, PROCESSOR_PRIME);
    public static final List<IntakeStates> exclusiveIntakeStates = List.of(PROCESSOR_PRIME);
    public static final List<IntakeStates> defaultClimbStates = List.of(CLIMB_PRIME);
    public static final List<IntakeStates> exclusiveClimbStates = List.of(CLIMB_LOCK, CLIMB_WINCH);

    private double angle;
    private double power;

    private IntakeStates(double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

    private IntakeStates(double angle) {
        this.angle = angle;
        this.power = 0;
    }

    private IntakeStates() {
        this.angle = 0;
        this.power = 0;
    }

    public double getAngle() {
        return angle;
    }

    public double getPower() {
        return power;
    }
}
