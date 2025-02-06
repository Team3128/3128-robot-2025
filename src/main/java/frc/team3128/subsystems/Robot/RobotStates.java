package frc.team3128.subsystems.Robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import io.vavr.collection.List;

public enum RobotStates {
    UNDEFINED(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, 0),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, 1),

    RPL1(ElevatorStates.L1, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, 0.5),
    RPL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, 0.5),
    RPL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, 0.5), 
    RPL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, 05),
    RSL1(ElevatorStates.L1, IntakeStates.NEUTRAL, ManipulatorStates.OUT, 0.5),
    RSL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.OUT, 0.5),
    RSL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.OUT, 0.5),
    RSL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.OUT, 0.5),
    
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL, 1),
    EJECT_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.EJECT_OUTTAKE, ManipulatorStates.NEUTRAL, 1),
    PROCESSOR_PRIME(ElevatorStates.NEUTRAL, IntakeStates.PROCESSOR_PRIME, ManipulatorStates.NEUTRAL, 0.5),
    PROCESSOR_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.PROCESSOR_OUTTAKE, ManipulatorStates.NEUTRAL, 0.5),
    
    CLIMB_PRIME(IntakeStates.CLIMB_PRIME, ClimberStates.CLIMB_PRIME),
    CLIMB(IntakeStates.CLIMB, ClimberStates.CLIMB);

    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private double throttle;
    private Translation2d position;

    public static final List<RobotStates> defaultElevatorStates = List.of(RPL1, RPL2, RPL3, RPL4);
    public static final List<RobotStates> exclusiveElevatorStates = List.of(RSL1, RSL2, RSL3, RSL4);
    public static final List<RobotStates> defaultIntakeStates = List.of(INTAKE, EJECT_OUTTAKE, PROCESSOR_PRIME);
    public static final List<RobotStates> exclusiveIntakeStates = List.of(PROCESSOR_PRIME);
    public static final List<RobotStates> defaultClimbStates = List.of(CLIMB_PRIME);
    public static final List<RobotStates> exclusiveClimbStates = List.of(CLIMB);

    public static final List<Pair<RobotStates, RobotStates>> coupledStates = List.of(
        Pair.of(RPL1, RSL1),
        Pair.of(RPL2, RSL2),
        Pair.of(RPL3, RSL3),
        Pair.of(RPL4, RSL4),
        Pair.of(UNDEFINED, NEUTRAL),
        Pair.of(PROCESSOR_PRIME, PROCESSOR_OUTTAKE),
        Pair.of(CLIMB_PRIME, CLIMB)
    );

    public static final List<RobotStates> defaultStates = List.of(NEUTRAL).appendAll(defaultElevatorStates).appendAll(defaultIntakeStates).appendAll(defaultClimbStates);
    public static final List<RobotStates> exclusiveStates = exclusiveElevatorStates.appendAll(exclusiveIntakeStates).appendAll(exclusiveClimbStates);

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, Translation2d position, double throttle) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.position = position;
        this.throttle = throttle;
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber) {
        this(elevator, intake, manipulator, climber, new Translation2d(), 1);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle) {
        this(elevator, intake, manipulator, climber, new Translation2d(), throttle);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle);
    }

    private RobotStates(IntakeStates intake, ClimberStates climber) {
        this(ElevatorStates.UNDEFINED, intake, ManipulatorStates.NEUTRAL, climber);
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
}