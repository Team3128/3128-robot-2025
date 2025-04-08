package frc.team3128.subsystems.Intake;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.Pair;
import io.vavr.collection.List;

public enum IntakeStates {
    UNDEFINED,
    NEUTRAL(-10, -.05),
    INTAKE(90, -0.8),
    EJECT_OUTTAKE(0, 1);
    
    public static final List<IntakeStates> defaultStates = List.of(NEUTRAL, INTAKE, EJECT_OUTTAKE);

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