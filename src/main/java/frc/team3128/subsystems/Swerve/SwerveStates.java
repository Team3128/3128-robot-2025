package frc.team3128.subsystems.Swerve;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import io.vavr.collection.List;

import static frc.team3128.Constants.DriveConstants.slow;
import static frc.team3128.Constants.DriveConstants.fast;

public enum SwerveStates {
    TELEOP(fast),
    REEF_ALIGN(slow),
    NEUTRAL(fast),
    FEED_ALIGN(slow),
    BARGE_ALIGN(slow);

    private double throttle;
    private Neutral neutral;


    public static final List<SwerveStates>  functionalStates = List.of(TELEOP,REEF_ALIGN, NEUTRAL, FEED_ALIGN, BARGE_ALIGN );
    
    private SwerveStates(double throttle) {
        this.throttle = throttle;
        this.neutral = Neutral.BRAKE;
    }

    private SwerveStates() {
        this.throttle = 0;
        this.neutral = Neutral.COAST;
    }

    public double getThrottle() {
        return this.throttle;
    }

    public Neutral getNeutral() {
        return this.neutral;
    }
}
