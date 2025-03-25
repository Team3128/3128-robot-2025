package frc.team3128.subsystems.Intake;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.Pair;
import io.vavr.collection.List;

public enum IntakeStates {
    UNDEFINED,
    NEUTRAL(0, 0.1),
    INTAKE(65, 0.8),
    EJECT_OUTTAKE(0, -0.8),
    HIGH_INTAKE(0, 0.8),
    CLIMB_PRIME(10),
    CLIMB(10);

    public static final List<IntakeStates> defaultStates = List.of(NEUTRAL, INTAKE, EJECT_OUTTAKE, CLIMB_PRIME, HIGH_INTAKE);
    public static final List<IntakeStates> exclusiveStates = List.of(CLIMB);

    public static final List<Pair<IntakeStates, IntakeStates>> coupledStates = List.of(
        Pair.of(CLIMB_PRIME, CLIMB)
    );

    private double angle;
    private double power;
    private Neutral neutral;

    private IntakeStates(double angle, double power) {
        this.angle = angle;
        this.power = power;
        this.neutral = Neutral.BRAKE;
    }

    private IntakeStates(double angle) {
        this(angle, 0);
    }

    private IntakeStates() {
        this.angle = 0;
        this.power = 0;
        this.neutral = Neutral.COAST;
    }

    public double getAngle() {
        return this.angle;
    }

    public double getPower() {
        return this.power;
    }

    public Neutral getNeutral() {
        return this.neutral;
    }
}
