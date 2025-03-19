package frc.team3128.subsystems.Robot;

import static frc.team3128.subsystems.Intake.IntakeStates.INTAKE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Led.LedStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import io.vavr.collection.List;
import static frc.team3128.Constants.DriveConstants.slow;
import static frc.team3128.Constants.DriveConstants.fast;

public enum RobotStates {
    
    FULL_NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.NEUTRAL, LedStates.DEFAULT),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.UNDEFINED, LedStates.DEFAULT),

    RPL1(ElevatorStates.LOW_L1, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true, LedStates.DEFAULT),
    RPL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true, LedStates.DEFAULT),
    RPL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true, LedStates.DEFAULT), 
    RPL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true, LedStates.DEFAULT),
    
    RSL1(ElevatorStates.HIGH_L1, IntakeStates.NEUTRAL, ManipulatorStates.OUT_L1, slow, true, LedStates.SCORE),
    RSL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true, LedStates.SCORE),
    RSL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true, LedStates.SCORE),
    RSL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true, LedStates.SCORE),

    TELE_HOLD(ElevatorStates.TELE_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false, LedStates.AUTO_HOLD),
    AUTO_HOLD(ElevatorStates.AUTO_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false, LedStates.AUTO_HOLD),
    
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL, LedStates.INTAKE),
    EJECT_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.EJECT_OUTTAKE, ManipulatorStates.NEUTRAL, LedStates.EJECT_OUTTAKE),
    HIGH_INTAKE(ElevatorStates.NEUTRAL, IntakeStates.HIGH_INTAKE, ManipulatorStates.NEUTRAL, LedStates.INTAKE),
    
    PRE_CLIMB_PRIME(ClimberStates.PRE_CLIMB_PRIME),
    CLIMB_PRIME(ElevatorStates.NEUTRAL, IntakeStates.CLIMB_PRIME, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB_PRIME, slow, LedStates.CLIMB_PRIME),
    CLIMB(ElevatorStates.NEUTRAL, IntakeStates.CLIMB, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB, slow, LedStates.CLIMB_PRIME);

    
    
    
    
    
    
    
    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private double throttle;
    private boolean waitForAutoEnabled;
    private LedStates leds;

    public static final List<RobotStates> defaultElevatorStates = List.of(RPL1, RPL2, RPL3, RPL4, TELE_HOLD, AUTO_HOLD);
    public static final List<RobotStates> exclusiveElevatorStates = List.of(RSL1, RSL2, RSL3, RSL4);
    public static final List<RobotStates> defaultIntakeStates = List.of(INTAKE, EJECT_OUTTAKE, HIGH_INTAKE);
    public static final List<RobotStates> defaultClimbStates = List.of(CLIMB_PRIME, PRE_CLIMB_PRIME);
    public static final List<RobotStates> exclusiveClimbStates = List.of(CLIMB);
    public static final List<RobotStates> neutralStates = List.of(NEUTRAL, FULL_NEUTRAL);

    public static final List<Pair<RobotStates, RobotStates>> coupledStates = List.of(
        // Pair.of(RPL1, RSL1),
        Pair.of(RPL2, RSL2),
        Pair.of(RPL3, RSL3),
        Pair.of(RPL4, RSL4),
        Pair.of(CLIMB_PRIME, CLIMB)
    );

    public static final List<RobotStates> defaultStates = List.of(NEUTRAL, FULL_NEUTRAL).appendAll(defaultElevatorStates).appendAll(defaultIntakeStates).appendAll(defaultClimbStates);
    public static final List<RobotStates> exclusiveStates = exclusiveElevatorStates.appendAll(exclusiveClimbStates);

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle, boolean waitForAutoEnabled, LedStates leds) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.throttle = throttle;
        this.waitForAutoEnabled = waitForAutoEnabled;
        this.leds = leds;
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle, LedStates leds) {
        this(elevator, intake, manipulator, climber, throttle, false, leds);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, LedStates leds) {
        this(elevator, intake, manipulator, climber, 1, false, leds);
    }


    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle, LedStates leds) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, false, leds);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, LedStates leds) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, 1, false, leds);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle, boolean waitForAutoEnabled, LedStates leds) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, waitForAutoEnabled, leds);
    }

    private RobotStates(ClimberStates climber, ElevatorStates elevator, double throttle, LedStates leds) {
        this(elevator, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, throttle, false, leds);
    }

    private RobotStates(ClimberStates climber) {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, fast, false, LedStates.UNDEFINED);
    }

    private RobotStates() {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, ClimberStates.UNDEFINED, LedStates.UNDEFINED);
    }

    public ElevatorStates getElevatorState() {
        return this.elevator;
    }

    public IntakeStates getIntakeState() {
        return this.intake;
    }

    public ManipulatorStates getManipulatorState() {
        return this.manipulator;
    }

    public ClimberStates getClimberState() {
        return this.climber;
    }

    public double getThrottle() {
        return this.throttle;
    }

    public boolean getWaitForAutoEnabled() {
        return this.waitForAutoEnabled;
    }

    public LedStates getLedState() {
        return this.leds;
    }
}