package frc.team3128.subsystems.Robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;

public enum RobotStates {
    RPL1(ElevatorStates.L1, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL),
    RPL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL),
    RPL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL),
    RPL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL),
    RSL1(ElevatorStates.L1, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    RSL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    RSL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    RSL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL),
    IDLE(ElevatorStates.IDLE, IntakeStates.IDLE, ManipulatorStates.IDLE),
    SOURCE(ElevatorStates.SOURCE, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    INDEXING(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.FORWARD),
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL),
    EJECT_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.EJECT_OUTTAKE, ManipulatorStates.NEUTRAL),
    PROCESSOR_PRIME(ElevatorStates.NEUTRAL, IntakeStates.PROCESSOR_PRIME, ManipulatorStates.NEUTRAL),
    PROCESSOR_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.PROCESSOR_OUTTAKE, ManipulatorStates.NEUTRAL),
    CLIMB_PRIME(IntakeStates.CLIMB_PRIME, ClimberStates.CLIMB_PRIME),
    CLIMB_LOCK(IntakeStates.CLIMB_LOCKED, ClimberStates.CLIMB_LOCKED),
    CLIMB_WINCH(IntakeStates.CLIMB_PRIME, ClimberStates.CLIMB_WINCH);

    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private Translation2d position;

    public static RobotStates[] reefPrimeStates = {RPL1, RPL2, RPL3, RPL4};
    public static RobotStates[] reefScoreStates = {RSL1, RSL2, RSL3, RSL4};
    public static RobotStates[] climbStates = {CLIMB_PRIME, CLIMB_LOCK, CLIMB_WINCH};

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, Translation2d position) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.position = position;
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber) {
        this(elevator, intake, manipulator, climber, new Translation2d());
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator) {
        this(elevator, intake, manipulator, ClimberStates.IDLE);
    }

    private RobotStates(IntakeStates intake, ClimberStates climber) {
        this(ElevatorStates.IDLE, intake, ManipulatorStates.NEUTRAL, climber);
    }

    private RobotStates() {
        this(ElevatorStates.IDLE, IntakeStates.IDLE, ManipulatorStates.IDLE, ClimberStates.IDLE);
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
}