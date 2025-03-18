package frc.team3128.subsystems.Swerve;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

public enum SwerveStates {
    UNDEFINED,
    NEUTRAL(1, false),
    AUTO_MOVEMENT(0.5, false),
    SCORING(0.3, true),
    CLIMBING(0.3, false);

    private double throttle;
    private boolean waitForAutoMove;

    public static final List<SwerveStates> functionalStates = List.of(SwerveStates.values());

    private SwerveStates(double throttle, boolean waitForAutoMove) {
        this.throttle = throttle;
        this.waitForAutoMove = waitForAutoMove;
    }

    private SwerveStates() {
        this(0, false);
    }

    public double getThrottle() {
        return this.throttle;
    }

    public boolean getWaitForAutoMove() {
        return this.waitForAutoMove;
    }
}
