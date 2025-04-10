package frc.team3128.subsystems.Robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import io.vavr.collection.List;
import static frc.team3128.Constants.DriveConstants.slow;
import static frc.team3128.Constants.DriveConstants.fast;

public enum RobotStates {
    
    FULL_NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.NEUTRAL),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.UNDEFINED),

    CORAL_PRIME_L2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, false),
    CORAL_PRIME_L3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, false), 
    CORAL_PRIME_L4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true),
    
    CORAL_SCORE_L2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),
    CORAL_SCORE_L3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),
    CORAL_SCORE_L4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),

    ALGAE_INTAKE_1(ElevatorStates.A1, IntakeStates.NEUTRAL, ManipulatorStates.IN_ALGAE, slow, false),
    ALGAE_INTAKE_2(ElevatorStates.A2, IntakeStates.NEUTRAL, ManipulatorStates.IN_ALGAE, slow, false),
    
    ALGAE_PRIME_2(ElevatorStates.AB, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, false),
    ALGAE_SCORE_2(ElevatorStates.AB, IntakeStates.NEUTRAL, ManipulatorStates.OUT_ALGAE, slow, true),

    TELE_HOLD(ElevatorStates.TELE_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false),
    AUTO_HOLD(ElevatorStates.AUTO_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false),
    
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL),
    OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.OUTTAKE, ManipulatorStates.NEUTRAL),
    
    PRE_CLIMB_PRIME(ClimberStates.PRE_CLIMB_PRIME),
    CLIMB_PRIME(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB_PRIME, slow),
    CLIMB(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB, slow);

    
    
    
    
    
    
    
    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private double throttle;
    private boolean waitForAutoEnabled;

    public static final List<RobotStates> defaultElevatorStates = List.of(CORAL_PRIME_L2, CORAL_PRIME_L3, CORAL_PRIME_L4, ALGAE_INTAKE_1, ALGAE_INTAKE_2, ALGAE_PRIME_2, TELE_HOLD, AUTO_HOLD);
    public static final List<RobotStates> exclusiveElevatorStates = List.of(CORAL_SCORE_L2, CORAL_SCORE_L3, CORAL_SCORE_L4, ALGAE_SCORE_2);
    public static final List<RobotStates> defaultIntakeStates = List.of(INTAKE, OUTTAKE);
    public static final List<RobotStates> defaultClimbStates = List.of(CLIMB_PRIME, PRE_CLIMB_PRIME);
    public static final List<RobotStates> exclusiveClimbStates = List.of(CLIMB);

    public static final List<Pair<RobotStates, RobotStates>> coupledStates = List.of(
        // Pair.of(CORAL_PRIME_L1, CORAL_SCORE_L1),
        Pair.of(CORAL_PRIME_L2, CORAL_SCORE_L2),
        Pair.of(CORAL_PRIME_L3, CORAL_SCORE_L3),
        Pair.of(CORAL_PRIME_L4, CORAL_SCORE_L4),
        Pair.of(CLIMB_PRIME, CLIMB)
    );

    public static final List<RobotStates> defaultStates = List.of(NEUTRAL, FULL_NEUTRAL).appendAll(defaultElevatorStates).appendAll(defaultIntakeStates).appendAll(defaultClimbStates);
    public static final List<RobotStates> exclusiveStates = exclusiveElevatorStates.appendAll(exclusiveClimbStates);

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle, boolean waitForAutoEnabled) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.throttle = throttle;
        this.waitForAutoEnabled = waitForAutoEnabled;
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle) {
        this(elevator, intake, manipulator, climber, throttle, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber) {
        this(elevator, intake, manipulator, climber, 1, false);
    }


    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, 1, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle, boolean waitForAutoEnabled) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, waitForAutoEnabled);
    }

    private RobotStates(ClimberStates climber, ElevatorStates elevator, double throttle) {
        this(elevator, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, throttle, false);
    }

    private RobotStates(ClimberStates climber) {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, fast, false);
    }

    private RobotStates() {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, ClimberStates.UNDEFINED);
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
}